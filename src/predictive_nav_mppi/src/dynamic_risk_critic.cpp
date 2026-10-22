#include "predictive_nav_mppi/dynamic_risk_critic.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi::critics
{

void DynamicRiskCritic::initialize()
{
  const auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "DynamicRiskCritic could not access its lifecycle node");
    return;
  }

  clock_ = node->get_clock();
  auto get_param = parameters_handler_->getParamGetter(name_);
  get_param(
    prediction_topic_, "prediction_topic",
    std::string{"/dynamic_obstacles/predictions"});
  get_param(
    expected_prediction_frame_, "expected_prediction_frame",
    std::string{"odom"});
  get_param(max_prediction_age_s_, "max_prediction_age_s", 0.5);
  get_param(max_future_time_skew_s_, "max_future_time_skew_s", 0.1);

  if (expected_prediction_frame_.empty() || !std::isfinite(max_prediction_age_s_) ||
    !std::isfinite(max_future_time_skew_s_) || max_prediction_age_s_ <= 0.0 ||
    max_future_time_skew_s_ < 0.0)
  {
    RCLCPP_ERROR(
      logger_, "DynamicRiskCritic has invalid prediction-validation parameters; "
      "expected_prediction_frame must be nonempty, max_prediction_age_s > 0, "
      "and max_future_time_skew_s >= 0");
    return;
  }

  prediction_subscription_ = node->create_subscription<PredictionArray>(
    prediction_topic_, rclcpp::QoS(10),
    [this](PredictionArray::ConstSharedPtr message) {
      prediction_callback(message);
    });

  RCLCPP_INFO(
    logger_, "Initialized %s as an MPPI prediction-cache critic (enabled: %s, topic: %s, "
    "expected_frame: %s, max_age: %.2f s, max_future_skew: %.2f s)",
    name_.c_str(), enabled_ ? "true" : "false", prediction_topic_.c_str(),
    expected_prediction_frame_.c_str(), max_prediction_age_s_, max_future_time_skew_s_);
}

void DynamicRiskCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // This class is intentionally read-only in steps 05--07. Future steps will
  // add dynamic risk with `data.costs(i) += ...`; they must not overwrite
  // costs already produced by Nav2's built-in critics.
  if (!clock_) {
    RCLCPP_ERROR(logger_, "DynamicRiskCritic has no ROS clock; skipping score");
    return;
  }

  const auto & shape = data.trajectories.x.shape();
  const auto prediction = latest_prediction_snapshot();
  if (!prediction) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "DynamicRiskCritic has no cached prediction yet; scoring remains unchanged");
    return;
  }

  double age_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_rejection_reason;
  if (!is_prediction_frame_usable(*prediction, clock_->now(), &age_s, &frame_rejection_reason)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "DynamicRiskCritic rejected cached prediction frame | reason=%s | frame=%s | "
      "age=%.3f s | trajectories=%zu",
      frame_rejection_reason.c_str(), prediction->header.frame_id.c_str(), age_s,
      prediction->trajectories.size());
    return;
  }

  std::size_t usable_trajectory_count = 0U;
  std::size_t rejected_trajectory_count = 0U;
  std::string first_trajectory_rejection_reason;
  for (const auto & trajectory : prediction->trajectories) {
    std::string trajectory_rejection_reason;
    if (is_predicted_trajectory_usable(trajectory, &trajectory_rejection_reason)) {
      ++usable_trajectory_count;
    } else {
      ++rejected_trajectory_count;
      if (first_trajectory_rejection_reason.empty()) {
        first_trajectory_rejection_reason = trajectory_rejection_reason;
      }
    }
  }

  if (rejected_trajectory_count > 0U) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "DynamicRiskCritic prediction validation | usable_trajectories=%zu | "
      "rejected_trajectories=%zu | first_reason=%s",
      usable_trajectory_count, rejected_trajectory_count,
      first_trajectory_rejection_reason.c_str());
  } else {
    RCLCPP_DEBUG_THROTTLE(
      logger_, *clock_, 2000,
      "DynamicRiskCritic accepted prediction frame: %zu candidate(s), %zu time step(s), "
      "%zu usable trajectory(s), age=%.3f s; scoring remains unchanged",
      static_cast<std::size_t>(shape[0]), static_cast<std::size_t>(shape[1]),
      usable_trajectory_count, age_s);
  }
}

void DynamicRiskCritic::prediction_callback(PredictionArray::ConstSharedPtr message)
{
  if (!message) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "DynamicRiskCritic received a null prediction message; cache is unchanged");
    return;
  }

  {
    std::scoped_lock lock(prediction_mutex_);
    latest_prediction_ = message;
  }

  // Receipt and cache replacement are intentionally separate from validation.
  // Freshness must be evaluated relative to each controller cycle, not only
  // when the callback happens to run.
  const rclcpp::Time stamp(message->header.stamp, RCL_ROS_TIME);
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 1000,
    "Cached prediction frame | topic=%s | stamp=%.3f s | frame=%s | trajectories=%zu",
    prediction_topic_.c_str(), stamp.seconds(), message->header.frame_id.c_str(),
    message->trajectories.size());
}

DynamicRiskCritic::PredictionArray::ConstSharedPtr
DynamicRiskCritic::latest_prediction_snapshot() const
{
  std::scoped_lock lock(prediction_mutex_);
  return latest_prediction_;
}

bool DynamicRiskCritic::is_prediction_frame_usable(
  const PredictionArray & prediction, const rclcpp::Time & now,
  double * age_s, std::string * rejection_reason) const
{
  const rclcpp::Time stamp(prediction.header.stamp, RCL_ROS_TIME);
  *age_s = (now - stamp).seconds();

  if (prediction.header.frame_id != expected_prediction_frame_) {
    *rejection_reason = "frame does not match expected_prediction_frame";
    return false;
  }
  if (stamp.nanoseconds() <= 0) {
    *rejection_reason = "timestamp is zero or negative";
    return false;
  }
  if (!std::isfinite(*age_s)) {
    *rejection_reason = "timestamp age is non-finite";
    return false;
  }
  if (*age_s > max_prediction_age_s_) {
    *rejection_reason = "timestamp is older than max_prediction_age_s";
    return false;
  }
  if (*age_s < -max_future_time_skew_s_) {
    *rejection_reason = "timestamp is implausibly ahead of the ROS clock";
    return false;
  }
  return true;
}

bool DynamicRiskCritic::is_predicted_trajectory_usable(
  const predictive_nav_msgs::msg::PredictedTrajectory & trajectory,
  std::string * rejection_reason) const
{
  if (trajectory.poses.empty()) {
    *rejection_reason = "trajectory has no poses";
    return false;
  }
  if (trajectory.poses.size() != trajectory.time_offsets.size()) {
    *rejection_reason = "pose and time_offsets sizes differ";
    return false;
  }

  double previous_offset_s = -1.0;
  for (std::size_t index = 0U; index < trajectory.poses.size(); ++index) {
    const auto & offset = trajectory.time_offsets[index];
    if (offset.sec < 0 || offset.nanosec >= 1000000000U) {
      *rejection_reason = "time offset is negative or has invalid nanoseconds";
      return false;
    }
    const double offset_s = static_cast<double>(offset.sec) +
      static_cast<double>(offset.nanosec) * 1e-9;
    if (!std::isfinite(offset_s) || offset_s <= previous_offset_s) {
      *rejection_reason = "time offsets are not strictly increasing and finite";
      return false;
    }
    previous_offset_s = offset_s;

    const auto & pose = trajectory.poses[index];
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
      !std::isfinite(pose.covariance[0]) || !std::isfinite(pose.covariance[1]) ||
      !std::isfinite(pose.covariance[6]) || !std::isfinite(pose.covariance[7]))
    {
      *rejection_reason = "pose x/y or 2D position covariance is non-finite";
      return false;
    }
  }
  return true;
}

}  // namespace mppi::critics

// Export the class symbol so pluginlib can construct it after it locates the
// matching XML declaration through the ament resource index.
PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicRiskCritic, mppi::critics::CriticFunction)
