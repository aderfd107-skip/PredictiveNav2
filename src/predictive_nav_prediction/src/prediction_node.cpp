#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "predictive_nav_msgs/msg/tracked_obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

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

}  // namespace

// A class derived from rclcpp::Node is a ROS 2 node written in C++.
// In Step 05 it observes the tracking output and owns the shared prediction
// time axis. It still does not calculate future positions or publish results.
class PredictionNode : public rclcpp::Node
{
public:
  PredictionNode()
  : Node("prediction_node")
  {
    prediction_horizon_s_ = declare_parameter<double>("prediction_horizon_s", 2.0);
    prediction_dt_s_ = declare_parameter<double>("prediction_dt_s", 0.2);
    prediction_time_offsets_s_ = make_prediction_time_offsets(
      prediction_horizon_s_, prediction_dt_s_);

    tracks_subscription_ = create_subscription<
      predictive_nav_msgs::msg::TrackedObstacleArray>(
      "/dynamic_obstacles/tracks",
      rclcpp::SensorDataQoS(),
      [this](predictive_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr message) {
        tracks_callback(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "prediction time axis | horizon=%.2f s | dt=%.2f s | point_count=%zu | offsets=%s",
      prediction_horizon_s_, prediction_dt_s_, prediction_time_offsets_s_.size(),
      format_prediction_time_offsets(prediction_time_offsets_s_).c_str());
    RCLCPP_INFO(
      get_logger(),
      "prediction_node started. Step 05 observes /dynamic_obstacles/tracks and defines time only.");
  }

private:
  void tracks_callback(
    const predictive_nav_msgs::msg::TrackedObstacleArray::ConstSharedPtr & message)
  {
    // Laser-derived track updates usually arrive around 10 Hz.  One log per
    // second is enough to inspect the data without flooding the terminal.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "track frame | stamp=%.3f s | frame=%s | track_count=%zu",
      rclcpp::Time(message->header.stamp).seconds(),
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
  }

  rclcpp::Subscription<predictive_nav_msgs::msg::TrackedObstacleArray>::SharedPtr
    tracks_subscription_;
  double prediction_horizon_s_{0.0};
  double prediction_dt_s_{0.0};
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
