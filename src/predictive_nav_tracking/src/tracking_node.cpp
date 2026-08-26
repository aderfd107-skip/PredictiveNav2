#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "predictive_nav_msgs/msg/obstacle_cluster_array.hpp"
#include "rclcpp/rclcpp.hpp"

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

    cluster_subscription_ = create_subscription<
      predictive_nav_msgs::msg::ObstacleClusterArray>(
      "/dynamic_obstacles/clusters",
      rclcpp::SensorDataQoS(),
      [this](predictive_nav_msgs::msg::ObstacleClusterArray::ConstSharedPtr message) {
        cluster_callback(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for obstacle-cluster messages on /dynamic_obstacles/clusters; "
      "accepting dt in (0, %.2f] s. Single-target debug reference=(%.2f, %.2f) m, max distance=%.2f m.",
      max_dt_s_,
      debug_target_x_m_,
      debug_target_y_m_,
      debug_target_max_distance_m_);
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

  void cluster_callback(
    const predictive_nav_msgs::msg::ObstacleClusterArray::ConstSharedPtr message)
  {
    ++message_count_;
    const rclcpp::Time current_stamp(message->header.stamp, RCL_ROS_TIME);
    const DeltaTimeResult delta_time = update_delta_time(current_stamp);
    const DebugClusterSelection debug_selection = select_debug_cluster(*message);
    const NaiveVelocityResult naive_velocity = update_naive_velocity(
      debug_selection, delta_time);

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
  }

  rclcpp::Subscription<predictive_nav_msgs::msg::ObstacleClusterArray>::SharedPtr
    cluster_subscription_;
  std::size_t message_count_{0U};
  // This vector is intentionally empty at step 04.  Later callbacks will
  // create, update and delete Track objects inside it.
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
};

}  // namespace predictive_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<predictive_nav::TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
