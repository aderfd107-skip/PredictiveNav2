#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "predictive_nav_msgs/msg/tracked_obstacle_array.hpp"
#include "predictive_nav_msgs/msg/predicted_trajectory.hpp"
#include "predictive_nav_msgs/msg/predicted_trajectory_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

// Create offsets such as [0.2, 0.4, ..., 2.0]. Offset zero is absent because
// the incoming Track already represents the state at the prediction start time.
std::vector<double> make_prediction_time_offsets(double horizon_s, double step_s)
{
  if (!std::isfinite(horizon_s) || !std::isfinite(step_s) || horizon_s <= 0.0 || step_s <= 0.0) {
    throw std::invalid_argument(
            "prediction_horizon_s and prediction_dt_s must both be finite values greater than zero");
  }

  constexpr double kComparisonEpsilon = 1e-9;
  constexpr std::size_t kMaximumPredictionPoints = 100U;
  std::vector<double> offsets_s;

  // Add regular samples that are strictly before the horizon.
  for (double offset_s = step_s; offset_s < horizon_s - kComparisonEpsilon; offset_s += step_s) {
    offsets_s.push_back(offset_s);
    if (offsets_s.size() >= kMaximumPredictionPoints) {
      throw std::invalid_argument(
              "prediction time axis would contain more than 100 points; increase prediction_dt_s or reduce prediction_horizon_s");
    }
  }

  // Always include the exact horizon. This also covers step_s > horizon_s,
  // where there is one useful prediction point: the horizon itself.
  offsets_s.push_back(horizon_s);
  return offsets_s;
}

std::string format_prediction_time_offsets(const std::vector<double> & offsets_s)
{
  std::ostringstream output;
  output << "[" << std::fixed << std::setprecision(2);
  for (std::size_t index = 0U; index < offsets_s.size(); ++index) {
    if (index != 0U) {
      output << ", ";
    }
    output << offsets_s[index];
  }
  output << "] s";
  return output.str();
}

struct PositionCovariance2D
{
  double xx_m2{0.0};
  double xy_m2{0.0};
  double yx_m2{0.0};
  double yy_m2{0.0};
};

struct PredictedPosition2D
{
  double time_offset_s{0.0};
  double x_m{0.0};
  double y_m{0.0};
  PositionCovariance2D position_covariance;
};

struct TrackValidationResult
{
  bool accepted{false};
  std::string reason;
};

// CV means Constant Velocity.  This function does not create a new
// measurement: it only propagates the current Track state along future time.
std::vector<PredictedPosition2D> propagate_cv_positions(
  const predictive_nav_msgs::msg::TrackedObstacle & track,
  const std::vector<double> & time_offsets_s,
  double acceleration_stddev_mps2)
{
  const double x_now_m = track.pose.pose.position.x;
  const double y_now_m = track.pose.pose.position.y;
  const double vx_mps = track.twist.twist.linear.x;
  const double vy_mps = track.twist.twist.linear.y;

  if (!std::isfinite(x_now_m) || !std::isfinite(y_now_m) ||
    !std::isfinite(vx_mps) || !std::isfinite(vy_mps) ||
    !std::isfinite(acceleration_stddev_mps2) || acceleration_stddev_mps2 < 0.0 ||
    !std::all_of(
      track.state_covariance.begin(), track.state_covariance.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return {};
  }

  const auto & covariance = track.state_covariance;
  const double acceleration_variance =
    acceleration_stddev_mps2 * acceleration_stddev_mps2;
  std::vector<PredictedPosition2D> predictions;
  predictions.reserve(time_offsets_s.size());
  for (const double time_offset_s : time_offsets_s) {
    const double time_squared = time_offset_s * time_offset_s;
    const double time_fourth = time_squared * time_squared;
    // Q_position is the position block of the CV model's process noise:
    // (t^4 / 4) * sigma_a^2 * I.  We model unknown x/y accelerations as
    // independent, so Q_xy and Q_yx are zero.
    const double process_position_variance =
      time_fourth * acceleration_variance / 4.0;

    // P_position(t) = P_pp + t(P_pv + P_vp) + t^2 P_vv + Q_position.
    // Indices use row-major [px, py, vx, vy] state covariance.
    const double future_xx_m2 = covariance[0] +
      time_offset_s * (covariance[2] + covariance[8]) +
      time_squared * covariance[10] + process_position_variance;
    const double future_xy_m2 = covariance[1] +
      time_offset_s * (covariance[3] + covariance[9]) +
      time_squared * covariance[11];
    const double future_yx_m2 = covariance[4] +
      time_offset_s * (covariance[6] + covariance[12]) +
      time_squared * covariance[14];
    const double future_yy_m2 = covariance[5] +
      time_offset_s * (covariance[7] + covariance[13]) +
      time_squared * covariance[15] + process_position_variance;
    // Mathematical covariance is symmetric.  Average the two off-diagonal
    // calculations to remove only floating-point round-off asymmetry.
    const double future_xy_symmetric_m2 = 0.5 * (future_xy_m2 + future_yx_m2);

    if (!std::isfinite(future_xx_m2) || !std::isfinite(future_xy_symmetric_m2) ||
      !std::isfinite(future_yy_m2))
    {
      return {};
    }

    predictions.push_back(PredictedPosition2D{
      time_offset_s,
      x_now_m + vx_mps * time_offset_s,
      y_now_m + vy_mps * time_offset_s,
      PositionCovariance2D{
        future_xx_m2, future_xy_symmetric_m2,
        future_xy_symmetric_m2, future_yy_m2}});
  }
  return predictions;
}

std::string format_cv_preview(const std::vector<PredictedPosition2D> & predictions)
{
  if (predictions.empty()) {
    return "<no finite prediction>";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(2);
  const auto append_point = [&output](const PredictedPosition2D & point) {
      output << "t=" << point.time_offset_s << " s -> (" << point.x_m << ", " << point.y_m <<
        ") m, Pdiag=(" << point.position_covariance.xx_m2 << ", " <<
        point.position_covariance.yy_m2 << ") m^2";
    };

  append_point(predictions.front());
  if (predictions.size() > 2U) {
    output << " | ";
    append_point(predictions[(predictions.size() - 1U) / 2U]);
  }
  if (predictions.size() > 1U) {
    output << " | ";
    append_point(predictions.back());
  }
  return output.str();
}

// The public message uses ROS's general 3D PoseWithCovariance even though this
// project predicts a planar (x, y) obstacle position. Keep unknown z/rotation
// variances large only on their diagonal; do not invent cross-correlations.
constexpr double kUnobservedVariance = 1.0e6;

bool make_predicted_trajectory(
  const predictive_nav_msgs::msg::TrackedObstacle & track,
  const std::vector<double> & time_offsets_s,
  double acceleration_stddev_mps2,
  predictive_nav_msgs::msg::PredictedTrajectory * trajectory,
  std::string * reason)
{
  const std::vector<PredictedPosition2D> predicted_positions = propagate_cv_positions(
    track, time_offsets_s, acceleration_stddev_mps2);
  if (predicted_positions.empty()) {
    *reason = "CV propagation returned no finite points";
    return false;
  }

  trajectory->track_id = track.track_id;
  trajectory->size = track.size;
  trajectory->confidence = track.confidence;
  trajectory->time_offsets.reserve(predicted_positions.size());
  trajectory->poses.reserve(predicted_positions.size());
  for (const PredictedPosition2D & point : predicted_positions) {
    trajectory->time_offsets.push_back(
      static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration::from_seconds(point.time_offset_s)));

    auto & pose = trajectory->poses.emplace_back();
    pose.pose.position.x = point.x_m;
    pose.pose.position.y = point.y_m;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.covariance.fill(0.0);
    for (const std::size_t index : {2U, 3U, 4U, 5U}) {
      pose.covariance[index * 6U + index] = kUnobservedVariance;
    }
    pose.covariance[0] = point.position_covariance.xx_m2;
    pose.covariance[1] = point.position_covariance.xy_m2;
    pose.covariance[6] = point.position_covariance.yx_m2;
    pose.covariance[7] = point.position_covariance.yy_m2;
  }
  return true;
}

bool is_predicted_trajectory_valid(
  const predictive_nav_msgs::msg::PredictedTrajectory & trajectory,
  std::string * reason)
{
  if (trajectory.track_id == 0U || trajectory.time_offsets.empty() ||
    trajectory.time_offsets.size() != trajectory.poses.size())
  {
    *reason = "track id is zero or time/pose arrays are empty or mismatched";
    return false;
  }
  if (!std::isfinite(trajectory.size.x) || !std::isfinite(trajectory.size.y) ||
    trajectory.size.x <= 0.0 || trajectory.size.y <= 0.0 ||
    !std::isfinite(trajectory.confidence))
  {
    *reason = "size or confidence is invalid";
    return false;
  }

  rclcpp::Duration previous_offset(0, 0U);
  for (std::size_t index = 0U; index < trajectory.poses.size(); ++index) {
    const rclcpp::Duration offset(trajectory.time_offsets[index]);
    const auto & pose = trajectory.poses[index];
    if (offset.nanoseconds() <= 0 || offset <= previous_offset ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
      !std::all_of(
        pose.covariance.begin(), pose.covariance.end(),
        [](double value) {return std::isfinite(value);}))
    {
      *reason = "time offsets or predicted pose/covariance is invalid";
      return false;
    }
    previous_offset = offset;
  }
  return true;
}

}  // namespace

// A class derived from rclcpp::Node is a ROS 2 node written in C++.
// In Step 06 it observes the tracking output, owns the shared prediction time
// axis, and calculates CV future positions. It still does not publish results.
class PredictionNode : public rclcpp::Node
{
public:
  PredictionNode()
  : Node("prediction_node")
  {
    prediction_horizon_s_ = declare_parameter<double>("prediction_horizon_s", 2.0);
    prediction_dt_s_ = declare_parameter<double>("prediction_dt_s", 0.2);
    prediction_acceleration_stddev_mps2_ = declare_parameter<double>(
      "prediction_acceleration_stddev_mps2", 1.0);
    max_track_message_age_s_ = declare_parameter<double>("max_track_message_age_s", 0.5);
    max_future_time_skew_s_ = declare_parameter<double>("max_future_time_skew_s", 0.1);
    const int configured_min_track_age_frames = declare_parameter<int>("min_track_age_frames", 3);
    const int configured_max_track_missed_frames = declare_parameter<int>(
      "max_track_missed_frames", 1);
    min_track_age_frames_ = configured_min_track_age_frames > 0 ?
      static_cast<std::size_t>(configured_min_track_age_frames) : 0U;
    max_track_missed_frames_ = configured_max_track_missed_frames >= 0 ?
      static_cast<std::size_t>(configured_max_track_missed_frames) : 0U;
    min_track_confidence_ = declare_parameter<double>("min_track_confidence", 0.5);
    max_track_speed_mps_ = declare_parameter<double>("max_track_speed_mps", 3.0);
    max_visual_uncertainty_diameter_m_ = declare_parameter<double>(
      "max_visual_uncertainty_diameter_m", 1.2);
    visualization_min_speed_mps_ = declare_parameter<double>(
      "visualization_min_speed_mps", 0.15);
    const int configured_visualization_max_trajectories = declare_parameter<int>(
      "visualization_max_trajectories", 3);
    visualization_max_trajectories_ = configured_visualization_max_trajectories > 0 ?
      static_cast<std::size_t>(configured_visualization_max_trajectories) : 0U;

    if (!std::isfinite(max_track_message_age_s_) || max_track_message_age_s_ <= 0.0 ||
      !std::isfinite(max_future_time_skew_s_) || max_future_time_skew_s_ < 0.0 ||
      !std::isfinite(min_track_confidence_) || min_track_confidence_ < 0.0 ||
      min_track_confidence_ > 1.0 || !std::isfinite(max_track_speed_mps_) ||
      max_track_speed_mps_ <= 0.0 ||
      !std::isfinite(max_visual_uncertainty_diameter_m_) ||
      max_visual_uncertainty_diameter_m_ <= 0.0 ||
      !std::isfinite(visualization_min_speed_mps_) || visualization_min_speed_mps_ < 0.0)
    {
      throw std::invalid_argument("Step 08 track-quality parameters have invalid values");
    }
    prediction_time_offsets_s_ = make_prediction_time_offsets(
      prediction_horizon_s_, prediction_dt_s_);

    tracks_subscription_ = create_subscription<
      predictive_nav_msgs::msg::TrackedObstacleArray>(
      "/dynamic_obstacles/tracks",
      rclcpp::SensorDataQoS(),
      [this](predictive_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr message) {
        tracks_callback(message);
      });
    predictions_publisher_ = create_publisher<
      predictive_nav_msgs::msg::PredictedTrajectoryArray>(
      "/dynamic_obstacles/predictions", rclcpp::QoS(10));
    prediction_markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dynamic_obstacles/prediction_markers", rclcpp::QoS(10));

    input_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {check_track_input_freshness();});

    RCLCPP_INFO(
      get_logger(),
      "prediction time axis | horizon=%.2f s | dt=%.2f s | point_count=%zu | offsets=%s",
      prediction_horizon_s_, prediction_dt_s_, prediction_time_offsets_s_.size(),
      format_prediction_time_offsets(prediction_time_offsets_s_).c_str());
    RCLCPP_INFO(
      get_logger(),
      "prediction process model | acceleration_stddev=%.2f m/s^2",
      prediction_acceleration_stddev_mps2_);
    RCLCPP_INFO(
      get_logger(),
      "track-quality gate | max_age=%.2f s | min_age=%zu frames | max_missed=%zu | "
      "min_confidence=%.2f | max_speed=%.2f m/s",
      max_track_message_age_s_, min_track_age_frames_, max_track_missed_frames_,
      min_track_confidence_, max_track_speed_mps_);
    RCLCPP_INFO(
      get_logger(),
      "RViz presentation filter | min_speed=%.2f m/s | max_trajectories=%zu | "
      "max_uncertainty_diameter=%.2f m (display only)",
      visualization_min_speed_mps_, visualization_max_trajectories_,
      max_visual_uncertainty_diameter_m_);
    RCLCPP_INFO(
      get_logger(),
      "prediction_node started. It publishes trajectories on /dynamic_obstacles/predictions "
      "and RViz-only markers on /dynamic_obstacles/prediction_markers; it never controls Nav2.");
  }

private:
  bool validate_track_frame(
    const predictive_nav_msgs::msg::TrackedObstacleArray & message,
    const rclcpp::Time & now, rclcpp::Time * stamp, std::string * reason) const
  {
    if (message.header.frame_id != "odom") {
      *reason = "frame is not odom";
      return false;
    }

    *stamp = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
    if (stamp->nanoseconds() <= 0) {
      *reason = "timestamp is zero or negative";
      return false;
    }

    const double age_s = (now - *stamp).seconds();
    if (age_s > max_track_message_age_s_) {
      *reason = "timestamp is older than max_track_message_age_s";
      return false;
    }
    if (age_s < -max_future_time_skew_s_) {
      *reason = "timestamp is implausibly ahead of the ROS clock";
      return false;
    }
    if (has_accepted_track_frame_ && *stamp <= last_accepted_track_stamp_) {
      *reason = "timestamp did not advance";
      return false;
    }
    return true;
  }

  TrackValidationResult validate_track(
    const predictive_nav_msgs::msg::TrackedObstacle & track) const
  {
    const auto & position = track.pose.pose.position;
    const auto & velocity = track.twist.twist.linear;
    const bool finite_state = std::isfinite(position.x) && std::isfinite(position.y) &&
      std::isfinite(velocity.x) && std::isfinite(velocity.y);
    const bool finite_covariance = std::all_of(
      track.state_covariance.begin(), track.state_covariance.end(),
      [](double value) {return std::isfinite(value);});
    const bool finite_size = std::isfinite(track.size.x) && std::isfinite(track.size.y);

    if (!finite_state || !finite_covariance || !finite_size ||
      !std::isfinite(track.confidence))
    {
      return TrackValidationResult{false, "state, covariance, size, or confidence is NaN/Inf"};
    }
    if (track.state_covariance[0] < 0.0 || track.state_covariance[5] < 0.0 ||
      track.state_covariance[10] < 0.0 || track.state_covariance[15] < 0.0)
    {
      return TrackValidationResult{false, "a state covariance diagonal is negative"};
    }
    if (track.size.x <= 0.0 || track.size.y <= 0.0) {
      return TrackValidationResult{false, "cluster size is not positive"};
    }
    if (track.age < min_track_age_frames_) {
      return TrackValidationResult{false, "track has not reached min_track_age_frames"};
    }
    if (track.missed_frames > max_track_missed_frames_) {
      return TrackValidationResult{false, "track missed too many consecutive frames"};
    }
    if (track.confidence < min_track_confidence_) {
      return TrackValidationResult{false, "track confidence is below min_track_confidence"};
    }
    if (std::hypot(velocity.x, velocity.y) > max_track_speed_mps_) {
      return TrackValidationResult{false, "track speed exceeds max_track_speed_mps"};
    }
    return TrackValidationResult{true, "accepted"};
  }

  void check_track_input_freshness()
  {
    if (!has_accepted_track_frame_) {
      return;
    }

    const double age_s = (get_clock()->now() - last_accepted_track_stamp_).seconds();
    if (age_s > max_track_message_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "track input is stale | last accepted stamp=%.3f s | age=%.3f s | "
        "holding prediction preview until a fresh frame arrives",
        last_accepted_track_stamp_.seconds(), age_s);
    }
  }

  // RViz understands MarkerArray directly, while downstream algorithms should
  // read PredictedTrajectoryArray.  Keeping the two outputs separate prevents
  // a visualization choice from becoming part of the prediction interface.
  void publish_prediction_markers(
    const predictive_nav_msgs::msg::PredictedTrajectoryArray & prediction_output,
    const std::vector<const predictive_nav_msgs::msg::TrackedObstacle *> & accepted_tracks)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    constexpr double kLineWidthM = 0.04;
    constexpr double kPointDiameterM = 0.10;
    constexpr double kUncertaintyLineWidthM = 0.025;
    constexpr std::size_t kEllipseSegmentCount = 32U;
    constexpr double kPi = 3.14159265358979323846;
    // A short lifetime automatically removes stale visualizations if tracking
    // input stops. The value is longer than the usual 10 Hz input period.
    const auto marker_lifetime = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(0.35));

    struct VisualizationCandidate
    {
      const predictive_nav_msgs::msg::PredictedTrajectory * trajectory{nullptr};
      double speed_mps{0.0};
      double score{0.0};
    };
    std::vector<VisualizationCandidate> candidates;
    candidates.reserve(prediction_output.trajectories.size());
    for (const auto & trajectory : prediction_output.trajectories) {
      const auto matching_track = std::find_if(
        accepted_tracks.begin(), accepted_tracks.end(),
        [&trajectory](const auto * track) {return track->track_id == trajectory.track_id;});
      if (trajectory.poses.empty() || matching_track == accepted_tracks.end()) {
        continue;
      }

      const auto & velocity = (*matching_track)->twist.twist.linear;
      const double speed_mps = std::hypot(velocity.x, velocity.y);
      if (speed_mps < visualization_min_speed_mps_) {
        continue;
      }
      candidates.push_back(VisualizationCandidate{
        &trajectory, speed_mps, speed_mps * (*matching_track)->confidence});
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const VisualizationCandidate & left, const VisualizationCandidate & right) {
        return left.score > right.score;
      });
    if (candidates.size() > visualization_max_trajectories_) {
      candidates.resize(visualization_max_trajectories_);
    }

    for (const auto & candidate : candidates) {
      const auto & trajectory = *candidate.trajectory;

      const int marker_id = static_cast<int>(trajectory.track_id);
      auto & line = marker_array.markers.emplace_back();
      line.header = prediction_output.header;
      line.ns = "prediction_trajectory";
      line.id = marker_id;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = kLineWidthM;
      line.color.g = 0.90F;
      line.color.b = 1.00F;
      line.color.a = 0.95F;
      line.lifetime = marker_lifetime;
      line.points.reserve(trajectory.poses.size());

      auto & points = marker_array.markers.emplace_back();
      points.header = prediction_output.header;
      points.ns = "prediction_points";
      points.id = marker_id;
      points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      points.action = visualization_msgs::msg::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.scale.x = kPointDiameterM;
      points.scale.y = kPointDiameterM;
      points.scale.z = kPointDiameterM;
      points.color.g = 0.90F;
      points.color.b = 1.00F;
      points.color.a = 0.95F;
      points.lifetime = marker_lifetime;
      points.points.reserve(trajectory.poses.size());

      for (const auto & predicted_pose : trajectory.poses) {
        auto & line_point = line.points.emplace_back();
        line_point.x = predicted_pose.pose.position.x;
        line_point.y = predicted_pose.pose.position.y;
        line_point.z = 0.06;
        auto & sphere_point = points.points.emplace_back();
        sphere_point.x = predicted_pose.pose.position.x;
        sphere_point.y = predicted_pose.pose.position.y;
        sphere_point.z = 0.08;
      }

      // Draw a true 2D covariance ellipse outline, not a translucent disk.
      // The outline is much easier to read in a project demonstration, and
      // Pxy determines its rotation instead of being silently ignored.
      const auto & final_pose = trajectory.poses.back();
      const double pxx_m2 = std::max(0.0, final_pose.covariance[0]);
      const double pxy_m2 = final_pose.covariance[1];
      const double pyy_m2 = std::max(0.0, final_pose.covariance[7]);
      const double trace_m2 = pxx_m2 + pyy_m2;
      const double discriminant_m2 = std::sqrt(std::max(
          0.0, (pxx_m2 - pyy_m2) * (pxx_m2 - pyy_m2) + 4.0 * pxy_m2 * pxy_m2));
      const double major_variance_m2 = std::max(0.0, 0.5 * (trace_m2 + discriminant_m2));
      const double minor_variance_m2 = std::max(0.0, 0.5 * (trace_m2 - discriminant_m2));
      const double major_axis_m = std::min(
        max_visual_uncertainty_diameter_m_ / 2.0, 2.0 * std::sqrt(major_variance_m2));
      const double minor_axis_m = std::min(
        max_visual_uncertainty_diameter_m_ / 2.0, 2.0 * std::sqrt(minor_variance_m2));
      const double ellipse_yaw_rad = 0.5 * std::atan2(2.0 * pxy_m2, pxx_m2 - pyy_m2);
      auto & uncertainty = marker_array.markers.emplace_back();
      uncertainty.header = prediction_output.header;
      uncertainty.ns = "prediction_horizon_uncertainty";
      uncertainty.id = marker_id;
      uncertainty.type = visualization_msgs::msg::Marker::LINE_STRIP;
      uncertainty.action = visualization_msgs::msg::Marker::ADD;
      uncertainty.pose.orientation.w = 1.0;
      uncertainty.scale.x = kUncertaintyLineWidthM;
      uncertainty.color.r = 0.75F;
      uncertainty.color.g = 0.35F;
      uncertainty.color.b = 1.00F;
      uncertainty.color.a = 0.90F;
      uncertainty.lifetime = marker_lifetime;
      uncertainty.points.reserve(kEllipseSegmentCount + 1U);
      for (std::size_t index = 0U; index <= kEllipseSegmentCount; ++index) {
        const double angle_rad =
          2.0 * kPi * static_cast<double>(index) / static_cast<double>(kEllipseSegmentCount);
        const double local_x_m = major_axis_m * std::cos(angle_rad);
        const double local_y_m = minor_axis_m * std::sin(angle_rad);
        auto & ellipse_point = uncertainty.points.emplace_back();
        ellipse_point.x = final_pose.pose.position.x +
          std::cos(ellipse_yaw_rad) * local_x_m - std::sin(ellipse_yaw_rad) * local_y_m;
        ellipse_point.y = final_pose.pose.position.y +
          std::sin(ellipse_yaw_rad) * local_x_m + std::cos(ellipse_yaw_rad) * local_y_m;
        ellipse_point.z = 0.10;
      }
    }

    prediction_markers_publisher_->publish(marker_array);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "published RViz presentation markers | displayed=%zu | trajectory_candidates=%zu",
      candidates.size(), prediction_output.trajectories.size());
  }

  void tracks_callback(
    const predictive_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr & message)
  {
    const rclcpp::Time now = get_clock()->now();
    rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
    std::string frame_rejection_reason;
    if (!validate_track_frame(*message, now, &stamp, &frame_rejection_reason)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "track frame rejected | frame=%s | reason=%s",
        message->header.frame_id.c_str(), frame_rejection_reason.c_str());
      return;
    }
    last_accepted_track_stamp_ = stamp;
    has_accepted_track_frame_ = true;

    // Laser-derived track updates usually arrive around 10 Hz.  One log per
    // second is enough to inspect the data without flooding the terminal.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "track frame | stamp=%.3f s | frame=%s | track_count=%zu",
      stamp.seconds(),
      message->header.frame_id.c_str(),
      message->obstacles.size());

    // Debug output is intentionally limited.  The message itself still
    // contains every track; only the human-facing terminal output is shortened.
    constexpr std::size_t kMaxTracksToLog = 3U;
    const std::size_t tracks_to_log = std::min(message->obstacles.size(), kMaxTracksToLog);
    for (std::size_t index = 0U; index < tracks_to_log; ++index) {
      const auto & track = message->obstacles[index];
      const auto & position = track.pose.pose.position;
      const auto & velocity = track.twist.twist.linear;
      const auto & position_covariance = track.pose.covariance;

      // [0, 1, 6, 7] form the x/y 2x2 block in ROS's row-major 6x6 pose
      // covariance array.  We only inspect it here; Step 07 will propagate it.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "  track[%zu] | id=%u | pos=(%.2f, %.2f) m | vel=(%.2f, %.2f) m/s | "
        "P_xy=[[%.4f, %.4f], [%.4f, %.4f]] m^2 | size=(%.2f, %.2f) m | "
        "age=%u | miss=%u | confidence=%.2f",
        index,
        track.track_id,
        position.x, position.y,
        velocity.x, velocity.y,
        position_covariance[0], position_covariance[1],
        position_covariance[6], position_covariance[7],
        track.size.x, track.size.y,
        track.age, track.missed_frames, track.confidence);
    }

    if (message->obstacles.size() > kMaxTracksToLog) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "  ... %zu additional track(s) omitted from this debug log.",
        message->obstacles.size() - kMaxTracksToLog);
    }

    std::vector<const predictive_nav_msgs::msg::TrackedObstacle *> accepted_tracks;
    accepted_tracks.reserve(message->obstacles.size());
    std::size_t rejected_track_count = 0U;
    std::string first_track_rejection_reason;
    for (const auto & track : message->obstacles) {
      const TrackValidationResult validation = validate_track(track);
      if (validation.accepted) {
        accepted_tracks.push_back(&track);
      } else {
        ++rejected_track_count;
        if (first_track_rejection_reason.empty()) {
          first_track_rejection_reason = validation.reason;
        }
      }
    }

    if (rejected_track_count > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "track quality gate | accepted=%zu | rejected=%zu | first_reason=%s",
        accepted_tracks.size(), rejected_track_count, first_track_rejection_reason.c_str());
    }

    // Publish one output frame for every fresh, correctly framed Track input.
    // An empty trajectories array has an explicit meaning: this input frame
    // arrived, but no Track was safe enough to predict.  That is different
    // from publishing nothing, which a downstream node cannot distinguish
    // from a dead prediction node without a timeout.
    predictive_nav_msgs::msg::PredictedTrajectoryArray prediction_output;
    prediction_output.header = message->header;
    prediction_output.trajectories.reserve(accepted_tracks.size());
    std::size_t propagation_rejection_count = 0U;
    std::string first_propagation_rejection_reason;
    for (const auto * track : accepted_tracks) {
      predictive_nav_msgs::msg::PredictedTrajectory trajectory;
      std::string rejection_reason;
      const bool built = make_predicted_trajectory(
        *track, prediction_time_offsets_s_, prediction_acceleration_stddev_mps2_,
        &trajectory, &rejection_reason);
      if (built && is_predicted_trajectory_valid(trajectory, &rejection_reason)) {
        prediction_output.trajectories.push_back(std::move(trajectory));
      } else {
        ++propagation_rejection_count;
        if (first_propagation_rejection_reason.empty()) {
          first_propagation_rejection_reason = rejection_reason;
        }
      }
    }
    predictions_publisher_->publish(prediction_output);
    publish_prediction_markers(prediction_output, accepted_tracks);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "published predictions | topic=/dynamic_obstacles/predictions | stamp=%.3f s | "
      "accepted_tracks=%zu | trajectories=%zu | points_per_trajectory=%zu | skipped=%zu",
      stamp.seconds(), accepted_tracks.size(), prediction_output.trajectories.size(),
      prediction_time_offsets_s_.size(), propagation_rejection_count);
    if (prediction_output.trajectories.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "published an empty prediction frame: no Track passed every validation step.");
    }
    if (propagation_rejection_count > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "prediction-output validation rejected %zu Track(s) | first_reason=%s",
        propagation_rejection_count, first_propagation_rejection_reason.c_str());
    }

    if (!accepted_tracks.empty()) {
      // This selection only keeps the debug log readable.  Every Track will
      // receive a prediction when the formal output is added in Step 09.
      const auto representative_track = std::max_element(
        accepted_tracks.begin(), accepted_tracks.end(),
        [](const auto * left, const auto * right) {
          const auto & left_velocity = left->twist.twist.linear;
          const auto & right_velocity = right->twist.twist.linear;
          return std::hypot(left_velocity.x, left_velocity.y) <
                 std::hypot(right_velocity.x, right_velocity.y);
        });
      const auto predictions = propagate_cv_positions(
        **representative_track, prediction_time_offsets_s_,
        prediction_acceleration_stddev_mps2_);

      if (predictions.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CV preview skipped for track id=%u because its state or covariance is invalid.",
          (*representative_track)->track_id);
      } else {
        const auto & position = (*representative_track)->pose.pose.position;
        const auto & velocity = (*representative_track)->twist.twist.linear;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CV preview | id=%u | now=(%.2f, %.2f) m | vel=(%.2f, %.2f) m/s | %s",
          (*representative_track)->track_id,
          position.x, position.y,
          velocity.x, velocity.y,
          format_cv_preview(predictions).c_str());
      }
    }
  }

  rclcpp::Subscription<predictive_nav_msgs::msg::TrackedObstacleArray>::SharedPtr
    tracks_subscription_;
  rclcpp::Publisher<predictive_nav_msgs::msg::PredictedTrajectoryArray>::SharedPtr
    predictions_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    prediction_markers_publisher_;
  rclcpp::TimerBase::SharedPtr input_watchdog_timer_;
  double prediction_horizon_s_{0.0};
  double prediction_dt_s_{0.0};
  double prediction_acceleration_stddev_mps2_{0.0};
  double max_track_message_age_s_{0.0};
  double max_future_time_skew_s_{0.0};
  std::size_t min_track_age_frames_{0U};
  std::size_t max_track_missed_frames_{0U};
  double min_track_confidence_{0.0};
  double max_track_speed_mps_{0.0};
  double max_visual_uncertainty_diameter_m_{0.0};
  double visualization_min_speed_mps_{0.0};
  std::size_t visualization_max_trajectories_{0U};
  bool has_accepted_track_frame_{false};
  rclcpp::Time last_accepted_track_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<double> prediction_time_offsets_s_;
};

int main(int argc, char * argv[])
{
  // Prepare ROS 2 communication for this process.
  rclcpp::init(argc, argv);

  // Construct the node, then let ROS 2 keep it alive and process callbacks.
  rclcpp::spin(std::make_shared<PredictionNode>());

  // This line runs after Ctrl+C stops spin().
  rclcpp::shutdown();
  return 0;
}
