#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "predictive_nav_msgs/msg/obstacle_cluster_array.hpp"
#include "predictive_nav_msgs/msg/tracked_obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace predictive_nav
{

// A cluster is a one-frame measurement.  A Track persists across callbacks
// and will later contain the filtered state of one physical object.
struct Track
{
  // The four positions in state have a fixed meaning throughout tracking.
  enum StateIndex : int
  {
    kPositionX = 0,
    kPositionY = 1,
    kVelocityX = 2,
    kVelocityY = 3
  };

  std::uint32_t track_id{0U};
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};//创建一个4x4的个单位矩阵
  double size_x_m{0.0};
  double size_y_m{0.0};
  rclcpp::Time first_observation_stamp{0};
  rclcpp::Time last_observation_stamp{0};
  std::size_t age{0U};
  std::size_t missed_frames{0U};
};

enum class DeltaTimeStatus
{
  kFirstMessage,
  kValid,
  kNonPositive,
  kTooLarge
};

struct DeltaTimeResult
{
  DeltaTimeStatus status{DeltaTimeStatus::kFirstMessage};
  double dt_s{0.0};
};

// Step 06 is deliberately a controlled, single-target experiment.  We do
// not use Gazebo ground truth: a cluster is selected from the same perception
// topic by asking which observed cluster is nearest to a chosen map region.
struct DebugClusterSelection
{
  const predictive_nav_msgs::msg::ObstacleCluster * cluster{nullptr};
  std::size_t cluster_index{0U};
  double distance_to_reference_m{std::numeric_limits<double>::infinity()};
};

struct NaiveVelocityResult
{
  bool available{false};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double speed_mps{0.0};
};

// Association answers one question only: which current-frame cluster is the
// most plausible continuation of an already predicted state?
struct DebugAssociationResult
{
  bool has_prediction{false};
  const predictive_nav_msgs::msg::ObstacleCluster * nearest_cluster{nullptr};
  std::size_t cluster_index{0U};
  double distance_m{std::numeric_limits<double>::infinity()};
  bool matched{false};
};

struct DebugKalmanUpdateResult
{
  bool applied{false};
  Eigen::Vector4d state_before{Eigen::Vector4d::Zero()};
  Eigen::Vector4d state_after{Eigen::Vector4d::Zero()};
  Eigen::Vector2d measurement{Eigen::Vector2d::Zero()};
  Eigen::Vector2d innovation{Eigen::Vector2d::Zero()};
  Eigen::Matrix<double, 4, 2> gain{Eigen::Matrix<double, 4, 2>::Zero()};
};

// Step 11 uses this only as a baseline association result.  It enforces that
// one cluster can update at most one Track in one frame.  It is deliberately
// not yet a Hungarian or Mahalanobis-distance association.
struct TrackAssociationResult
{
  std::vector<int> matched_cluster_for_track;
  std::vector<bool> matched_cluster;
  std::size_t matched_pair_count{0U};
};

struct TrackLifecycleStats
{
  std::size_t predicted_track_count{0U};
  std::size_t matched_track_count{0U};
  std::size_t missed_track_count{0U};
  std::size_t born_track_count{0U};
  std::size_t removed_track_count{0U};
};

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_node")
  {
    max_dt_s_ = declare_parameter<double>("max_dt_s", 0.50);
    debug_target_x_m_ = declare_parameter<double>("debug_target_x_m", 2.25);
    debug_target_y_m_ = declare_parameter<double>("debug_target_y_m", -2.85);
    debug_target_max_distance_m_ = declare_parameter<double>(
      "debug_target_max_distance_m", 1.00);
    initial_position_stddev_m_ = declare_parameter<double>(
      "initial_position_stddev_m", 0.20);
    initial_velocity_stddev_mps_ = declare_parameter<double>(
      "initial_velocity_stddev_mps", 1.00);
    debug_max_initial_speed_mps_ = declare_parameter<double>(
      "debug_max_initial_speed_mps", 0.80);
    process_acceleration_stddev_mps2_ = declare_parameter<double>(
      "process_acceleration_stddev_mps2", 1.00);
    association_gate_m_ = declare_parameter<double>("association_gate_m", 0.40);
    measurement_position_stddev_m_ = declare_parameter<double>(
      "measurement_position_stddev_m", 0.15);
    const std::int64_t configured_max_missed_frames =
      declare_parameter<int>("max_missed_frames", 5);
    max_missed_frames_ = configured_max_missed_frames > 0 ?
      static_cast<std::size_t>(configured_max_missed_frames) : 0U;

    cluster_subscription_ = create_subscription<
      predictive_nav_msgs::msg::ObstacleClusterArray>(
      "/dynamic_obstacles/clusters",
      rclcpp::SensorDataQoS(),
      [this](predictive_nav_msgs::msg::ObstacleClusterArray::ConstSharedPtr message) {
        cluster_callback(message);
      });
    tracks_publisher_ = create_publisher<predictive_nav_msgs::msg::TrackedObstacleArray>(
      "/dynamic_obstacles/tracks", rclcpp::SensorDataQoS());
    track_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dynamic_obstacles/track_markers", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for obstacle-cluster messages on /dynamic_obstacles/clusters; "
      "accepting dt in (0, %.2f] s. Single-target debug reference=(%.2f, %.2f) m, max distance=%.2f m. "
      "Initial CV stddev=(position=%.2f m, velocity=%.2f m/s), max seed speed=%.2f m/s, "
      "association gate=%.2f m, measurement position stddev=%.2f m, max missed frames=%zu.",
      max_dt_s_,
      debug_target_x_m_,
      debug_target_y_m_,
      debug_target_max_distance_m_,
      initial_position_stddev_m_,
      initial_velocity_stddev_mps_,
      debug_max_initial_speed_mps_,
      association_gate_m_,
      measurement_position_stddev_m_,
      max_missed_frames_);
    RCLCPP_INFO(
      get_logger(),
      "Real Track states will be published on /dynamic_obstacles/tracks with SensorDataQoS.");
    RCLCPP_INFO(
      get_logger(),
      "RViz-only Track markers will be published on /dynamic_obstacles/track_markers.");
  }

private:
  DeltaTimeResult update_delta_time(const rclcpp::Time & current_stamp)
  {
    if (!has_previous_cluster_stamp_) {
      previous_cluster_stamp_ = current_stamp;
      has_previous_cluster_stamp_ = true;
      return DeltaTimeResult{DeltaTimeStatus::kFirstMessage, 0.0};
    }

    const double dt_s = (current_stamp - previous_cluster_stamp_).seconds();
    // Reset the reference even after a time jump, so one bad stamp cannot
    // make every following frame appear invalid.
    previous_cluster_stamp_ = current_stamp;

    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      ++non_positive_dt_count_;
      return DeltaTimeResult{DeltaTimeStatus::kNonPositive, dt_s};
    }

    if (dt_s > max_dt_s_) {
      ++too_large_dt_count_;
      return DeltaTimeResult{DeltaTimeStatus::kTooLarge, dt_s};
    }

    return DeltaTimeResult{DeltaTimeStatus::kValid, dt_s};
  }

  const char * delta_time_status_name(DeltaTimeStatus status) const
  {
    switch (status) {
      case DeltaTimeStatus::kFirstMessage:
        return "first";
      case DeltaTimeStatus::kValid:
        return "valid";
      case DeltaTimeStatus::kNonPositive:
        return "non_positive";
      case DeltaTimeStatus::kTooLarge:
        return "too_large";
    }
    return "unknown";
  }

  DebugClusterSelection select_debug_cluster(
    const predictive_nav_msgs::msg::ObstacleClusterArray & message) const
  {
    DebugClusterSelection selection;

    for (std::size_t index = 0U; index < message.clusters.size(); ++index) {
      const auto & candidate = message.clusters[index];
      const double distance_m = std::hypot(
        static_cast<double>(candidate.centroid.x) - debug_target_x_m_,
        static_cast<double>(candidate.centroid.y) - debug_target_y_m_);

      if (distance_m < selection.distance_to_reference_m) {
        selection.cluster = &candidate;
        selection.cluster_index = index;
        selection.distance_to_reference_m = distance_m;
      }
    }

    if (selection.distance_to_reference_m > debug_target_max_distance_m_) {
      selection.cluster = nullptr;
    }
    return selection;
  }

  NaiveVelocityResult update_naive_velocity(
    const DebugClusterSelection & selection,
    const DeltaTimeResult & delta_time)
  {
    NaiveVelocityResult result;

    // A finite-difference velocity needs two consecutive, valid observations.
    // Losing either one resets the small experiment instead of joining points
    // that may belong to different physical objects.
    if (selection.cluster == nullptr || delta_time.status != DeltaTimeStatus::kValid) {
      has_previous_debug_observation_ = false;
      return result;
    }

    const double current_x_m = static_cast<double>(selection.cluster->centroid.x);
    const double current_y_m = static_cast<double>(selection.cluster->centroid.y);

    if (!has_previous_debug_observation_) {
      previous_debug_x_m_ = current_x_m;
      previous_debug_y_m_ = current_y_m;
      has_previous_debug_observation_ = true;
      return result;
    }

    result.vx_mps = (current_x_m - previous_debug_x_m_) / delta_time.dt_s;
    result.vy_mps = (current_y_m - previous_debug_y_m_) / delta_time.dt_s;
    result.speed_mps = std::hypot(result.vx_mps, result.vy_mps);
    result.available = true;

    previous_debug_x_m_ = current_x_m;
    previous_debug_y_m_ = current_y_m;
    return result;
  }

  Eigen::Matrix4d make_initial_cv_covariance() const
  {
    // A covariance diagonal stores variances, so each standard deviation is
    // squared.  Off-diagonal terms start at zero: initially we assume no
    // known correlation between position and velocity errors.
    const double position_variance = initial_position_stddev_m_ * initial_position_stddev_m_;
    const double velocity_variance =
      initial_velocity_stddev_mps_ * initial_velocity_stddev_mps_;

    Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
    covariance(Track::kPositionX, Track::kPositionX) = position_variance;
    covariance(Track::kPositionY, Track::kPositionY) = position_variance;
    covariance(Track::kVelocityX, Track::kVelocityX) = velocity_variance;
    covariance(Track::kVelocityY, Track::kVelocityY) = velocity_variance;
    return covariance;
  }

  Eigen::Matrix4d make_cv_transition_matrix(double dt_s) const
  {
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(Track::kPositionX, Track::kVelocityX) = dt_s;
    transition(Track::kPositionY, Track::kVelocityY) = dt_s;
    return transition;
  }

  Eigen::Matrix4d make_cv_process_noise(double dt_s) const
  {
    // This is the standard 2D constant-velocity model with unknown white
    // acceleration.  Larger acceleration uncertainty lets P grow faster.
    const double dt2 = dt_s * dt_s;
    const double dt3 = dt2 * dt_s;
    const double dt4 = dt2 * dt2;
    const double acceleration_variance =
      process_acceleration_stddev_mps2_ * process_acceleration_stddev_mps2_;

    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    process_noise(Track::kPositionX, Track::kPositionX) = dt4 / 4.0;
    process_noise(Track::kPositionY, Track::kPositionY) = dt4 / 4.0;
    process_noise(Track::kPositionX, Track::kVelocityX) = dt3 / 2.0;
    process_noise(Track::kPositionY, Track::kVelocityY) = dt3 / 2.0;
    process_noise(Track::kVelocityX, Track::kPositionX) = dt3 / 2.0;
    process_noise(Track::kVelocityY, Track::kPositionY) = dt3 / 2.0;
    process_noise(Track::kVelocityX, Track::kVelocityX) = dt2;
    process_noise(Track::kVelocityY, Track::kVelocityY) = dt2;
    return acceleration_variance * process_noise;
  }

  Track make_new_track(
    const predictive_nav_msgs::msg::ObstacleCluster & cluster,
    const rclcpp::Time & current_stamp)
  {
    Track track;
    track.track_id = next_track_id_++;
    // A new cluster is only one observation.  Starting its velocity at zero
    // avoids treating the Step 06 finite difference as a trustworthy speed.
    track.state <<
      static_cast<double>(cluster.centroid.x),
      static_cast<double>(cluster.centroid.y),
      0.0,
      0.0;
    track.covariance = make_initial_cv_covariance();
    track.size_x_m = static_cast<double>(cluster.size_x_m);
    track.size_y_m = static_cast<double>(cluster.size_y_m);
    track.first_observation_stamp = current_stamp;
    track.last_observation_stamp = current_stamp;
    track.age = 1U;
    track.missed_frames = 0U;
    return track;
  }

  bool predict_track(Track & track, double dt_s) const
  {
    const Eigen::Matrix4d transition = make_cv_transition_matrix(dt_s);
    track.state = transition * track.state;
    track.covariance =
      transition * track.covariance * transition.transpose() + make_cv_process_noise(dt_s);
    track.covariance = 0.5 * (track.covariance + track.covariance.transpose());
    return track.state.allFinite() && track.covariance.allFinite();
  }

  std::size_t predict_all_tracks(double dt_s)
  {
    const std::size_t before_count = tracks_.size();
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this, dt_s](Track & track) {
          if (predict_track(track, dt_s)) {
            return false;
          }
          RCLCPP_ERROR(
            get_logger(),
            "Track id=%u produced a non-finite prediction and was discarded.",
            static_cast<unsigned int>(track.track_id));
          return true;
        }),
      tracks_.end());
    return before_count - tracks_.size();
  }

  TrackAssociationResult associate_tracks_one_to_one(
    const predictive_nav_msgs::msg::ObstacleClusterArray & message) const
  {
    struct Candidate
    {
      std::size_t track_index;
      std::size_t cluster_index;
      double distance_m;
    };

    TrackAssociationResult result;
    result.matched_cluster_for_track.assign(tracks_.size(), -1);
    result.matched_cluster.assign(message.clusters.size(), false);
    std::vector<Candidate> candidates;

    for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
      const Track & track = tracks_[track_index];
      for (std::size_t cluster_index = 0U; cluster_index < message.clusters.size(); ++cluster_index) {
        const auto & cluster = message.clusters[cluster_index];
        const double x_m = static_cast<double>(cluster.centroid.x);
        const double y_m = static_cast<double>(cluster.centroid.y);
        if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
          continue;
        }

        const double distance_m = std::hypot(
          x_m - track.state(Track::kPositionX),
          y_m - track.state(Track::kPositionY));
        if (distance_m <= association_gate_m_) {
          candidates.push_back(Candidate{track_index, cluster_index, distance_m});
        }
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & left, const Candidate & right) {
        return left.distance_m < right.distance_m;
      });

    // Greedily accept the shortest remaining pair.  This is one-to-one, so a
    // cluster cannot be used to update two Tracks in this frame.
    for (const Candidate & candidate : candidates) {
      if (result.matched_cluster_for_track[candidate.track_index] >= 0 ||
        result.matched_cluster[candidate.cluster_index])
      {
        continue;
      }
      result.matched_cluster_for_track[candidate.track_index] =
        static_cast<int>(candidate.cluster_index);
      result.matched_cluster[candidate.cluster_index] = true;
      ++result.matched_pair_count;
    }
    return result;
  }

  bool update_track_from_cluster(
    Track & track,
    const predictive_nav_msgs::msg::ObstacleCluster & cluster,
    const rclcpp::Time & current_stamp) const
  {
    Eigen::Matrix<double, 2, 4> measurement_model = Eigen::Matrix<double, 2, 4>::Zero();
    measurement_model(0, Track::kPositionX) = 1.0;
    measurement_model(1, Track::kPositionY) = 1.0;

    Eigen::Vector2d measurement;
    measurement << static_cast<double>(cluster.centroid.x), static_cast<double>(cluster.centroid.y);
    const Eigen::Vector2d innovation = measurement - measurement_model * track.state;
    const double measurement_variance =
      measurement_position_stddev_m_ * measurement_position_stddev_m_;
    const Eigen::Matrix2d measurement_noise =
      measurement_variance * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d innovation_covariance =
      measurement_model * track.covariance * measurement_model.transpose() + measurement_noise;

    Eigen::LDLT<Eigen::Matrix2d> solver(innovation_covariance);
    if (solver.info() != Eigen::Success) {
      return false;
    }

    // K = P H^T S^-1.  Solve S × X = H × P instead of explicitly creating S^-1.
    const Eigen::Matrix<double, 2, 4> gain_transpose =
      solver.solve(measurement_model * track.covariance);
    if (solver.info() != Eigen::Success || !gain_transpose.allFinite()) {
      return false;
    }
    const Eigen::Matrix<double, 4, 2> gain = gain_transpose.transpose();
    const Eigen::Matrix4d covariance_before = track.covariance;
    track.state = track.state + gain * innovation;
    const Eigen::Matrix4d residual_transform =
      Eigen::Matrix4d::Identity() - gain * measurement_model;
    track.covariance =
      residual_transform * covariance_before * residual_transform.transpose() +
      gain * measurement_noise * gain.transpose();
    track.covariance = 0.5 * (track.covariance + track.covariance.transpose());

    if (!track.state.allFinite() || !track.covariance.allFinite()) {
      return false;
    }

    track.size_x_m = static_cast<double>(cluster.size_x_m);
    track.size_y_m = static_cast<double>(cluster.size_y_m);
    track.last_observation_stamp = current_stamp;
    return true;
  }

  TrackLifecycleStats update_track_lifecycle(
    const predictive_nav_msgs::msg::ObstacleClusterArray & message,
    const DeltaTimeResult & delta_time,
    const rclcpp::Time & current_stamp)
  {
    TrackLifecycleStats stats;
    if (message.header.frame_id != "odom" ||
      (delta_time.status != DeltaTimeStatus::kFirstMessage &&
      delta_time.status != DeltaTimeStatus::kValid))
    {
      return stats;
    }

    if (delta_time.status == DeltaTimeStatus::kValid) {
      const std::size_t discarded_non_finite = predict_all_tracks(delta_time.dt_s);
      stats.predicted_track_count = tracks_.size();
      stats.removed_track_count += discarded_non_finite;
    }

    TrackAssociationResult association;
    if (delta_time.status == DeltaTimeStatus::kValid) {
      association = associate_tracks_one_to_one(message);
    } else {
      association.matched_cluster_for_track.assign(tracks_.size(), -1);
      association.matched_cluster.assign(message.clusters.size(), false);
    }

    for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
      Track & track = tracks_[track_index];
      const int cluster_index = association.matched_cluster_for_track[track_index];
      if (cluster_index >= 0 && update_track_from_cluster(
          track, message.clusters[static_cast<std::size_t>(cluster_index)], current_stamp))
      {
        track.missed_frames = 0U;
        ++track.age;
        ++stats.matched_track_count;
      } else if (delta_time.status == DeltaTimeStatus::kValid) {
        ++track.missed_frames;
        ++track.age;
        ++stats.missed_track_count;
      }
    }

    const std::size_t before_expiry_count = tracks_.size();
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const Track & track) {
          if (track.missed_frames <= max_missed_frames_) {
            return false;
          }
          RCLCPP_INFO(
            get_logger(),
            "Track expired | id=%u | age=%zu frames | missed_frames=%zu exceeds max_missed_frames=%zu",
            static_cast<unsigned int>(track.track_id),
            track.age,
            track.missed_frames,
            max_missed_frames_);
          return true;
        }),
      tracks_.end());
    stats.removed_track_count += before_expiry_count - tracks_.size();

    // Every unmatched finite cluster is a possible new physical object.  The
    // track begins with velocity zero and gains a velocity only after future
    // predict-associate-update cycles.
    for (std::size_t cluster_index = 0U; cluster_index < message.clusters.size(); ++cluster_index) {
      if (association.matched_cluster[cluster_index]) {
        continue;
      }
      const auto & cluster = message.clusters[cluster_index];
      if (!std::isfinite(static_cast<double>(cluster.centroid.x)) ||
        !std::isfinite(static_cast<double>(cluster.centroid.y)))
      {
        continue;
      }
      Track track = make_new_track(cluster, current_stamp);
      RCLCPP_INFO(
        get_logger(),
        "Track born | id=%u | cluster_index=%zu | position=(%.2f, %.2f) m | velocity=(0.00, 0.00) m/s",
        static_cast<unsigned int>(track.track_id),
        cluster_index,
        track.state(Track::kPositionX),
        track.state(Track::kPositionY));
      tracks_.push_back(track);
      ++stats.born_track_count;
    }
    return stats;
  }

  std::uint32_t saturate_to_uint32(std::size_t value) const
  {
    const std::size_t largest_uint32 =
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max());
    return static_cast<std::uint32_t>(std::min(value, largest_uint32));
  }

  void publish_tracks(const predictive_nav_msgs::msg::ObstacleClusterArray & cluster_frame)
  {
    predictive_nav_msgs::msg::TrackedObstacleArray output;
    // The state of every output obstacle represents this cluster frame's
    // measurement time, not the later wall-clock time when the callback runs.
    output.header = cluster_frame.header;
    output.obstacles.reserve(tracks_.size());

    // PoseWithCovariance/TwistWithCovariance are standard 3D ROS messages.
    // Our 2D LiDAR tracker does not observe z, roll, pitch, yaw, vz or angular
    // velocity.  Only their *diagonal variances* are made large; cross terms
    // stay zero because “unobserved” does not imply a huge correlation with
    // x/y or vx/vy.
    constexpr double kUnobservedVariance = 1.0e6;
    for (const Track & track : tracks_) {
      auto & obstacle = output.obstacles.emplace_back();
      obstacle.track_id = track.track_id;

      obstacle.pose.pose.position.x = track.state(Track::kPositionX);
      obstacle.pose.pose.position.y = track.state(Track::kPositionY);
      obstacle.pose.pose.position.z = 0.0;
      obstacle.pose.pose.orientation.w = 1.0;
      obstacle.pose.covariance.fill(0.0);
      // 6x6 row-major diagonal: z(2,2), roll(3,3), pitch(4,4), yaw(5,5).
      for (const std::size_t index : {2U, 3U, 4U, 5U}) {
        obstacle.pose.covariance[index * 6U + index] = kUnobservedVariance;
      }
      // ROS covariance arrays are 6x6 row-major: x/y are positions 0/1.
      obstacle.pose.covariance[0] = track.covariance(Track::kPositionX, Track::kPositionX);
      obstacle.pose.covariance[1] = track.covariance(Track::kPositionX, Track::kPositionY);
      obstacle.pose.covariance[6] = track.covariance(Track::kPositionY, Track::kPositionX);
      obstacle.pose.covariance[7] = track.covariance(Track::kPositionY, Track::kPositionY);

      obstacle.twist.twist.linear.x = track.state(Track::kVelocityX);
      obstacle.twist.twist.linear.y = track.state(Track::kVelocityY);
      obstacle.twist.twist.linear.z = 0.0;
      obstacle.twist.covariance.fill(0.0);
      // 6x6 row-major diagonal: vz(2,2), wx(3,3), wy(4,4), wz(5,5).
      for (const std::size_t index : {2U, 3U, 4U, 5U}) {
        obstacle.twist.covariance[index * 6U + index] = kUnobservedVariance;
      }
      // In TwistWithCovariance, vx/vy are entries 0/1 in the same layout.
      obstacle.twist.covariance[0] = track.covariance(Track::kVelocityX, Track::kVelocityX);
      obstacle.twist.covariance[1] = track.covariance(Track::kVelocityX, Track::kVelocityY);
      obstacle.twist.covariance[6] = track.covariance(Track::kVelocityY, Track::kVelocityX);
      obstacle.twist.covariance[7] = track.covariance(Track::kVelocityY, Track::kVelocityY);

      obstacle.size.x = track.size_x_m;
      obstacle.size.y = track.size_y_m;
      obstacle.size.z = 0.0;
      obstacle.age = saturate_to_uint32(track.age);
      obstacle.missed_frames = saturate_to_uint32(track.missed_frames);
      // This is deliberately only an observation-freshness heuristic.  It is
      // not a calibrated probability that this physical identity is correct.
      obstacle.confidence = static_cast<float>(
        1.0 / (1.0 + static_cast<double>(track.missed_frames)));
    }

    tracks_publisher_->publish(output);
    if (message_count_ % 10U == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "published tracks | topic=/dynamic_obstacles/tracks | frame=%s | stamp=%d.%09u | count=%zu",
        output.header.frame_id.c_str(),
        static_cast<int>(output.header.stamp.sec),
        static_cast<unsigned int>(output.header.stamp.nanosec),
        output.obstacles.size());
    }
  }

  void publish_track_markers(
    const predictive_nav_msgs::msg::ObstacleClusterArray & cluster_frame)
  {
    // These Markers are a visual explanation of the public tracks topic.
    // Prediction modules must consume /dynamic_obstacles/tracks instead;
    // MarkerArray is intentionally a debug-only interface for RViz.
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear_previous;
    clear_previous.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear_previous);

    constexpr double kBoxLineWidthM = 0.045;
    constexpr double kVelocityArrowSeconds = 0.70;
    constexpr double kMaximumArrowLengthM = 1.20;
    constexpr double kMinimumArrowSpeedMps = 0.03;

    for (const Track & track : tracks_) {
      const bool observed_this_frame = track.missed_frames == 0U;
      const float red = observed_this_frame ? 0.15F : 1.00F;
      const float green = observed_this_frame ? 1.00F : 0.55F;
      const float blue = observed_this_frame ? 0.25F : 0.05F;
      const int marker_id = static_cast<int>(track.track_id);
      const double x = track.state(Track::kPositionX);
      const double y = track.state(Track::kPositionY);
      const double half_x = std::max(0.05, track.size_x_m * 0.5);
      const double half_y = std::max(0.05, track.size_y_m * 0.5);

      visualization_msgs::msg::Marker box;
      box.header = cluster_frame.header;
      box.ns = "tracked_obstacle_box";
      box.id = marker_id;
      box.type = visualization_msgs::msg::Marker::LINE_STRIP;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.scale.x = kBoxLineWidthM;
      box.color.r = red;
      box.color.g = green;
      box.color.b = blue;
      box.color.a = 1.0F;
      box.pose.orientation.w = 1.0;
      box.points.resize(5U);
      box.points[0].x = x - half_x;
      box.points[0].y = y - half_y;
      box.points[1].x = x + half_x;
      box.points[1].y = y - half_y;
      box.points[2].x = x + half_x;
      box.points[2].y = y + half_y;
      box.points[3].x = x - half_x;
      box.points[3].y = y + half_y;
      box.points[4] = box.points[0];
      output.markers.push_back(box);

      const double velocity_x = track.state(Track::kVelocityX);
      const double velocity_y = track.state(Track::kVelocityY);
      const double speed_mps = std::hypot(velocity_x, velocity_y);
      if (speed_mps >= kMinimumArrowSpeedMps) {
        const double arrow_length = std::min(
          kMaximumArrowLengthM, speed_mps * kVelocityArrowSeconds);
        visualization_msgs::msg::Marker arrow;
        arrow.header = cluster_frame.header;
        arrow.ns = "tracked_obstacle_velocity";
        arrow.id = marker_id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.045;
        arrow.scale.y = 0.10;
        arrow.scale.z = 0.13;
        arrow.color.r = 1.0F;
        arrow.color.g = 0.95F;
        arrow.color.b = 0.10F;
        arrow.color.a = 1.0F;
        arrow.points.resize(2U);
        arrow.points[0].x = x;
        arrow.points[0].y = y;
        arrow.points[0].z = 0.08;
        arrow.points[1].x = x + arrow_length * velocity_x / speed_mps;
        arrow.points[1].y = y + arrow_length * velocity_y / speed_mps;
        arrow.points[1].z = 0.08;
        output.markers.push_back(arrow);
      }

      visualization_msgs::msg::Marker label;
      label.header = cluster_frame.header;
      label.ns = "tracked_obstacle_label";
      label.id = marker_id;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = x;
      label.pose.position.y = y;
      label.pose.position.z = 0.35;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.24;
      label.color.r = red;
      label.color.g = green;
      label.color.b = blue;
      label.color.a = 1.0F;
      std::ostringstream text;
      text << "ID " << track.track_id << " | v=" << std::fixed << std::setprecision(2)
           << speed_mps << " m/s | miss=" << track.missed_frames;
      label.text = text.str();
      output.markers.push_back(label);
    }

    track_markers_publisher_->publish(output);
  }

  void initialize_debug_cv_state(
    const DebugClusterSelection & selection,
    const NaiveVelocityResult & velocity,
    const DeltaTimeResult & delta_time,
    const rclcpp::Time & current_stamp)
  {
    // This is deliberately not a real track yet.  It lets us inspect the
    // exact state/covariance layout before real tracks are associated and
    // later receive Kalman measurement updates.
    if (has_debug_cv_state_ || selection.cluster == nullptr ||
      delta_time.status != DeltaTimeStatus::kValid || !velocity.available ||
      !std::isfinite(velocity.speed_mps) || velocity.speed_mps > debug_max_initial_speed_mps_)
    {
      return;
    }

    debug_cv_state_.track_id = 0U;  // 0 means "teaching/debug state", not a public track ID.
    debug_cv_state_.state <<
      static_cast<double>(selection.cluster->centroid.x),
      static_cast<double>(selection.cluster->centroid.y),
      velocity.vx_mps,
      velocity.vy_mps;
    debug_cv_state_.covariance = make_initial_cv_covariance();
    debug_cv_state_.size_x_m = static_cast<double>(selection.cluster->size_x_m);
    debug_cv_state_.size_y_m = static_cast<double>(selection.cluster->size_y_m);
    debug_cv_state_.first_observation_stamp = current_stamp;
    debug_cv_state_.last_observation_stamp = current_stamp;
    debug_cv_state_.age = 1U;
    debug_cv_state_.missed_frames = 0U;
    has_debug_cv_state_ = true;

    RCLCPP_INFO(
      get_logger(),
      "CV state initialized (debug only) | x=[px=%.2f, py=%.2f, vx=%.2f, vy=%.2f] | "
      "P_diag=[%.3f, %.3f, %.3f, %.3f] | cluster_index=%zu | seed_speed=%.2f m/s",
      debug_cv_state_.state(Track::kPositionX),
      debug_cv_state_.state(Track::kPositionY),
      debug_cv_state_.state(Track::kVelocityX),
      debug_cv_state_.state(Track::kVelocityY),
      debug_cv_state_.covariance(Track::kPositionX, Track::kPositionX),
      debug_cv_state_.covariance(Track::kPositionY, Track::kPositionY),
      debug_cv_state_.covariance(Track::kVelocityX, Track::kVelocityX),
      debug_cv_state_.covariance(Track::kVelocityY, Track::kVelocityY),
      selection.cluster_index,
      velocity.speed_mps);
  }

  void predict_debug_cv_state(const DeltaTimeResult & delta_time)
  {
    if (!has_debug_cv_state_ || delta_time.status != DeltaTimeStatus::kValid) {
      return;
    }

    const Eigen::Vector4d state_before = debug_cv_state_.state;
    const Eigen::Matrix4d transition = make_cv_transition_matrix(delta_time.dt_s);
    const Eigen::Matrix4d process_noise = make_cv_process_noise(delta_time.dt_s);

    debug_cv_state_.state = transition * debug_cv_state_.state;
    debug_cv_state_.covariance =
      transition * debug_cv_state_.covariance * transition.transpose() + process_noise;
    // Floating-point matrix multiplication can introduce a tiny asymmetry;
    // covariance must mathematically remain symmetric.
    debug_cv_state_.covariance =
      0.5 * (debug_cv_state_.covariance + debug_cv_state_.covariance.transpose());

    if (!debug_cv_state_.state.allFinite() || !debug_cv_state_.covariance.allFinite()) {
      RCLCPP_ERROR(
        get_logger(),
        "CV prediction produced a non-finite state; discard the debug state and wait for a new valid seed.");
      has_debug_cv_state_ = false;
      return;
    }

    if (message_count_ % 10U == 0U) {
      RCLCPP_INFO(
        get_logger(),
        "CV predict (debug only) | dt=%.3f s | before=(%.2f, %.2f, %.2f, %.2f) | "
        "after=(%.2f, %.2f, %.2f, %.2f) | P_diag=(%.3f, %.3f, %.3f, %.3f)",
        delta_time.dt_s,
        state_before(Track::kPositionX),
        state_before(Track::kPositionY),
        state_before(Track::kVelocityX),
        state_before(Track::kVelocityY),
        debug_cv_state_.state(Track::kPositionX),
        debug_cv_state_.state(Track::kPositionY),
        debug_cv_state_.state(Track::kVelocityX),
        debug_cv_state_.state(Track::kVelocityY),
        debug_cv_state_.covariance(Track::kPositionX, Track::kPositionX),
        debug_cv_state_.covariance(Track::kPositionY, Track::kPositionY),
        debug_cv_state_.covariance(Track::kVelocityX, Track::kVelocityX),
        debug_cv_state_.covariance(Track::kVelocityY, Track::kVelocityY));
    }
  }

  DebugAssociationResult associate_debug_prediction(
    const predictive_nav_msgs::msg::ObstacleClusterArray & message) const
  {
    DebugAssociationResult result;
    result.has_prediction = has_debug_cv_state_;
    if (!result.has_prediction) {
      return result;
    }

    const double predicted_x_m = debug_cv_state_.state(Track::kPositionX);
    const double predicted_y_m = debug_cv_state_.state(Track::kPositionY);

    for (std::size_t index = 0U; index < message.clusters.size(); ++index) {
      const auto & candidate = message.clusters[index];
      const double candidate_x_m = static_cast<double>(candidate.centroid.x);
      const double candidate_y_m = static_cast<double>(candidate.centroid.y);
      if (!std::isfinite(candidate_x_m) || !std::isfinite(candidate_y_m)) {
        continue;
      }

      const double distance_m = std::hypot(
        candidate_x_m - predicted_x_m,
        candidate_y_m - predicted_y_m);
      if (distance_m < result.distance_m) {
        result.nearest_cluster = &candidate;
        result.cluster_index = index;
        result.distance_m = distance_m;
      }
    }

    result.matched =
      result.nearest_cluster != nullptr && result.distance_m <= association_gate_m_;
    return result;
  }

  void log_debug_nearest_neighbor_association(const DebugAssociationResult & association) const
  {
    if (!association.has_prediction) {
      RCLCPP_INFO(
        get_logger(),
        "nearest-neighbor association (debug only) | waiting for a seeded CV prediction");
      return;
    }

    const double predicted_x_m = debug_cv_state_.state(Track::kPositionX);
    const double predicted_y_m = debug_cv_state_.state(Track::kPositionY);
    if (association.nearest_cluster == nullptr) {
      RCLCPP_INFO(
        get_logger(),
        "nearest-neighbor association (debug only) | predicted=(%.2f, %.2f) m | "
        "no finite cluster candidate",
        predicted_x_m,
        predicted_y_m);
      return;
    }

    if (!association.matched) {
      RCLCPP_INFO(
        get_logger(),
        "nearest-neighbor association (debug only) | predicted=(%.2f, %.2f) m | "
        "nearest_cluster_index=%zu | centroid=(%.2f, %.2f) m | distance=%.2f m | "
        "gate=%.2f m | result=rejected_by_gate",
        predicted_x_m,
        predicted_y_m,
        association.cluster_index,
        association.nearest_cluster->centroid.x,
        association.nearest_cluster->centroid.y,
        association.distance_m,
        association_gate_m_);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "nearest-neighbor association (debug only) | predicted=(%.2f, %.2f) m | "
      "matched_cluster_index=%zu | centroid=(%.2f, %.2f) m | distance=%.2f m | gate=%.2f m",
      predicted_x_m,
      predicted_y_m,
      association.cluster_index,
      association.nearest_cluster->centroid.x,
      association.nearest_cluster->centroid.y,
      association.distance_m,
      association_gate_m_);
  }

  DebugKalmanUpdateResult update_debug_cv_state(
    const DebugAssociationResult & association,
    const DeltaTimeResult & delta_time,
    const rclcpp::Time & current_stamp)
  {
    DebugKalmanUpdateResult result;
    if (!has_debug_cv_state_ || !association.matched ||
      delta_time.status != DeltaTimeStatus::kValid)
    {
      return result;
    }

    // H says what the LiDAR measurement can see: position x and y, but not
    // velocity directly.  The filter can still adjust velocity because P
    // records the position-velocity relationship created during prediction.
    Eigen::Matrix<double, 2, 4> measurement_model = Eigen::Matrix<double, 2, 4>::Zero();
    measurement_model(0, Track::kPositionX) = 1.0;
    measurement_model(1, Track::kPositionY) = 1.0;

    result.state_before = debug_cv_state_.state;
    result.measurement <<
      static_cast<double>(association.nearest_cluster->centroid.x),
      static_cast<double>(association.nearest_cluster->centroid.y);
    result.innovation = result.measurement - measurement_model * result.state_before;

    const double measurement_variance =
      measurement_position_stddev_m_ * measurement_position_stddev_m_;
    const Eigen::Matrix2d measurement_noise =
      measurement_variance * Eigen::Matrix2d::Identity();
    const Eigen::Matrix2d innovation_covariance =
      measurement_model * debug_cv_state_.covariance * measurement_model.transpose() +
      measurement_noise;

    Eigen::LDLT<Eigen::Matrix2d> solver(innovation_covariance);
    if (solver.info() != Eigen::Success) {
      RCLCPP_ERROR(get_logger(), "Kalman update could not factor the innovation covariance; skip update.");
      return result;
    }

    const Eigen::Matrix<double, 2, 4> gain_transpose =
      solver.solve(measurement_model * debug_cv_state_.covariance);
    if (solver.info() != Eigen::Success || !gain_transpose.allFinite()) {
      RCLCPP_ERROR(get_logger(), "Kalman update could not solve the innovation covariance; skip update.");
      return result;
    }

    result.gain = gain_transpose.transpose();
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d covariance_before = debug_cv_state_.covariance;
    debug_cv_state_.state = result.state_before + result.gain * result.innovation;
    // Joseph form is algebraically equivalent to the familiar (I-KH)P but
    // preserves a symmetric, non-negative covariance better in floating point.
    const Eigen::Matrix4d residual_transform = identity - result.gain * measurement_model;
    debug_cv_state_.covariance =
      residual_transform * covariance_before * residual_transform.transpose() +
      result.gain * measurement_noise * result.gain.transpose();
    debug_cv_state_.covariance =
      0.5 * (debug_cv_state_.covariance + debug_cv_state_.covariance.transpose());

    if (!debug_cv_state_.state.allFinite() || !debug_cv_state_.covariance.allFinite()) {
      RCLCPP_ERROR(
        get_logger(),
        "Kalman update produced a non-finite state; discard the debug state and wait for a new valid seed.");
      has_debug_cv_state_ = false;
      return result;
    }

    debug_cv_state_.size_x_m = static_cast<double>(association.nearest_cluster->size_x_m);
    debug_cv_state_.size_y_m = static_cast<double>(association.nearest_cluster->size_y_m);
    debug_cv_state_.last_observation_stamp = current_stamp;
    ++debug_cv_state_.age;
    result.state_after = debug_cv_state_.state;
    result.applied = true;
    return result;
  }

  void log_debug_kalman_update(
    const DebugAssociationResult & association,
    const DebugKalmanUpdateResult & update) const
  {
    if (!has_debug_cv_state_) {
      RCLCPP_INFO(get_logger(), "Kalman update (debug only) | waiting for a seeded CV state");
      return;
    }

    if (!association.matched) {
      RCLCPP_INFO(
        get_logger(),
        "Kalman update (debug only) | skipped because this frame has no gated association");
      return;
    }

    if (!update.applied) {
      RCLCPP_WARN(
        get_logger(),
        "Kalman update (debug only) | matched a cluster but rejected the numerical update");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Kalman update (debug only) | measurement=(%.2f, %.2f) m | innovation=(%.2f, %.2f) m | "
      "before=(%.2f, %.2f, %.2f, %.2f) | after=(%.2f, %.2f, %.2f, %.2f) | "
      "K_velocity=(%.3f, %.3f)",
      update.measurement(0),
      update.measurement(1),
      update.innovation(0),
      update.innovation(1),
      update.state_before(Track::kPositionX),
      update.state_before(Track::kPositionY),
      update.state_before(Track::kVelocityX),
      update.state_before(Track::kVelocityY),
      update.state_after(Track::kPositionX),
      update.state_after(Track::kPositionY),
      update.state_after(Track::kVelocityX),
      update.state_after(Track::kVelocityY),
      update.gain(Track::kVelocityX, 0),
      update.gain(Track::kVelocityY, 1));
  }

  void log_single_target_debug(
    const DebugClusterSelection & selection,
    const NaiveVelocityResult & velocity) const
  {
    if (selection.cluster == nullptr) {
      RCLCPP_INFO(
        get_logger(),
        "single-target debug | reference=(%.2f, %.2f) m | no cluster within %.2f m",
        debug_target_x_m_,
        debug_target_y_m_,
        debug_target_max_distance_m_);
      return;
    }

    if (!velocity.available) {
      RCLCPP_INFO(
        get_logger(),
        "single-target debug | cluster_index=%zu | centroid=(%.2f, %.2f) m | "
        "reference_distance=%.2f m | velocity=warming_up",
        selection.cluster_index,
        selection.cluster->centroid.x,
        selection.cluster->centroid.y,
        selection.distance_to_reference_m);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "single-target debug | cluster_index=%zu | centroid=(%.2f, %.2f) m | "
      "reference_distance=%.2f m | naive_velocity=(%.2f, %.2f) m/s | speed=%.2f m/s",
      selection.cluster_index,
      selection.cluster->centroid.x,
      selection.cluster->centroid.y,
      selection.distance_to_reference_m,
      velocity.vx_mps,
      velocity.vy_mps,
      velocity.speed_mps);
  }

  void log_track_lifecycle(const TrackLifecycleStats & stats) const
  {
    RCLCPP_INFO(
      get_logger(),
      "track lifecycle | predicted=%zu | matched=%zu | missed=%zu | born=%zu | removed=%zu | active=%zu",
      stats.predicted_track_count,
      stats.matched_track_count,
      stats.missed_track_count,
      stats.born_track_count,
      stats.removed_track_count,
      tracks_.size());

    // Print a few records, rather than all of them, so the beginner can see
    // that IDs persist without turning the terminal into an unreadable table.
    constexpr std::size_t kMaxLoggedTracks = 3U;
    const std::size_t logged_track_count = std::min(kMaxLoggedTracks, tracks_.size());
    for (std::size_t index = 0U; index < logged_track_count; ++index) {
      const Track & track = tracks_[index];
      RCLCPP_INFO(
        get_logger(),
        "track[%zu] | id=%u | position=(%.2f, %.2f) m | velocity=(%.2f, %.2f) m/s | "
        "age=%zu | missed=%zu",
        index,
        static_cast<unsigned int>(track.track_id),
        track.state(Track::kPositionX),
        track.state(Track::kPositionY),
        track.state(Track::kVelocityX),
        track.state(Track::kVelocityY),
        track.age,
        track.missed_frames);
    }
  }

  void cluster_callback(
    const predictive_nav_msgs::msg::ObstacleClusterArray::ConstSharedPtr message)
  {
    ++message_count_;
    const rclcpp::Time current_stamp(message->header.stamp, RCL_ROS_TIME);
    const DeltaTimeResult delta_time = update_delta_time(current_stamp);
    const TrackLifecycleStats lifecycle_stats = update_track_lifecycle(
      *message, delta_time, current_stamp);
    const bool can_publish_tracks = message->header.frame_id == "odom" &&
      (delta_time.status == DeltaTimeStatus::kFirstMessage ||
      delta_time.status == DeltaTimeStatus::kValid);
    if (can_publish_tracks) {
      publish_tracks(*message);
      publish_track_markers(*message);
    }
    const DebugClusterSelection debug_selection = select_debug_cluster(*message);
    const NaiveVelocityResult naive_velocity = update_naive_velocity(
      debug_selection, delta_time);
    predict_debug_cv_state(delta_time);
    const DebugAssociationResult debug_association = associate_debug_prediction(*message);
    const DebugKalmanUpdateResult kalman_update = update_debug_cv_state(
      debug_association, delta_time, current_stamp);
    initialize_debug_cv_state(debug_selection, naive_velocity, delta_time, current_stamp);

    if (delta_time.status == DeltaTimeStatus::kNonPositive) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting this frame for future tracking because dt=%.6f s is not positive; "
        "reset the time reference and wait for the next frame.",
        delta_time.dt_s);
    } else if (delta_time.status == DeltaTimeStatus::kTooLarge) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting this frame for future tracking because dt=%.3f s exceeds max_dt_s=%.3f s; "
        "this can happen after a pause or time jump.",
        delta_time.dt_s,
        max_dt_s_);
    }

    // The perception node normally publishes about ten frames per second.
    // Logging every tenth frame proves the stream continues without making
    // the terminal unreadable.
    if (message_count_ % 10U != 0U) {
      return;
    }

    if (message->header.frame_id != "odom") {
      RCLCPP_WARN(
        get_logger(),
        "Received cluster frame=%s, but tracking expects odom. Do not estimate motion in lidar_link.",
        message->header.frame_id.c_str());
    }

    if (message->clusters.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "cluster frame #%zu | frame=%s | stamp=%d.%09u | clusters=0 | "
        "dt=%.3f s (%s) | active_tracks=%zu | next_track_id=%u | "
        "bad_dt(non_positive=%zu, too_large=%zu)",
        message_count_,
        message->header.frame_id.c_str(),
        static_cast<int>(message->header.stamp.sec),
        static_cast<unsigned int>(message->header.stamp.nanosec),
        delta_time.dt_s,
        delta_time_status_name(delta_time.status),
        tracks_.size(),
        static_cast<unsigned int>(next_track_id_),
        non_positive_dt_count_,
        too_large_dt_count_);
      log_single_target_debug(debug_selection, naive_velocity);
      log_debug_nearest_neighbor_association(debug_association);
      log_debug_kalman_update(debug_association, kalman_update);
      log_track_lifecycle(lifecycle_stats);
      return;
    }

    const auto & first_cluster = message->clusters.front();
    RCLCPP_INFO(
      get_logger(),
      "cluster frame #%zu | frame=%s | stamp=%d.%09u | clusters=%zu | "
      "first_centroid=(%.2f, %.2f) m | size=(%.2f, %.2f) m | points=%u | "
      "dt=%.3f s (%s) | active_tracks=%zu | next_track_id=%u | "
      "bad_dt(non_positive=%zu, too_large=%zu)",
      message_count_,
      message->header.frame_id.c_str(),
      static_cast<int>(message->header.stamp.sec),
      static_cast<unsigned int>(message->header.stamp.nanosec),
      message->clusters.size(),
      first_cluster.centroid.x,
      first_cluster.centroid.y,
      static_cast<double>(first_cluster.size_x_m),
      static_cast<double>(first_cluster.size_y_m),
      static_cast<unsigned int>(first_cluster.point_count),
      delta_time.dt_s,
      delta_time_status_name(delta_time.status),
      tracks_.size(),
      static_cast<unsigned int>(next_track_id_),
      non_positive_dt_count_,
      too_large_dt_count_);
    log_single_target_debug(debug_selection, naive_velocity);
    log_debug_nearest_neighbor_association(debug_association);
    log_debug_kalman_update(debug_association, kalman_update);
    log_track_lifecycle(lifecycle_stats);
  }

  rclcpp::Subscription<predictive_nav_msgs::msg::ObstacleClusterArray>::SharedPtr
    cluster_subscription_;
  rclcpp::Publisher<predictive_nav_msgs::msg::TrackedObstacleArray>::SharedPtr
    tracks_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    track_markers_publisher_;
  std::size_t message_count_{0U};
  // From step 11 onward, this is the real persistent track collection.  The
  // debug_cv_state_ below remains only as a teaching trace for steps 06--10.
  std::vector<Track> tracks_;
  std::uint32_t next_track_id_{1U};
  rclcpp::Time previous_cluster_stamp_{0};
  bool has_previous_cluster_stamp_{false};
  double max_dt_s_{0.50};
  std::size_t non_positive_dt_count_{0U};
  std::size_t too_large_dt_count_{0U};
  double debug_target_x_m_{2.25};
  double debug_target_y_m_{-2.85};
  double debug_target_max_distance_m_{1.00};
  double previous_debug_x_m_{0.0};
  double previous_debug_y_m_{0.0};
  bool has_previous_debug_observation_{false};
  double initial_position_stddev_m_{0.20};
  double initial_velocity_stddev_mps_{1.00};
  double debug_max_initial_speed_mps_{0.80};
  double process_acceleration_stddev_mps2_{1.00};
  double association_gate_m_{0.40};
  double measurement_position_stddev_m_{0.15};
  std::size_t max_missed_frames_{5U};
  Track debug_cv_state_{};
  bool has_debug_cv_state_{false};
};

}  // namespace predictive_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<predictive_nav::TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
