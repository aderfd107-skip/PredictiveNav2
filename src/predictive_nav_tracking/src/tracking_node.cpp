#include <cmath>
#include <cstddef>
#include <cstdint>
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

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_node")
  {
    max_dt_s_ = declare_parameter<double>("max_dt_s", 0.50);

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
      "accepting dt in (0, %.2f] s.",
      max_dt_s_);
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

  void cluster_callback(
    const predictive_nav_msgs::msg::ObstacleClusterArray::ConstSharedPtr message)
  {
    ++message_count_;
    const rclcpp::Time current_stamp(message->header.stamp, RCL_ROS_TIME);
    const DeltaTimeResult delta_time = update_delta_time(current_stamp);

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
};

}  // namespace predictive_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<predictive_nav::TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
