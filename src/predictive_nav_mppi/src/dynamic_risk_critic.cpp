#include "predictive_nav_mppi/dynamic_risk_critic.hpp"

#include <cstddef>

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
  RCLCPP_INFO(
    logger_, "Initialized %s as an empty MPPI critic (enabled: %s)",
    name_.c_str(), enabled_ ? "true" : "false");
}

void DynamicRiskCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // This class is intentionally read-only in step 05.  Future steps will
  // add dynamic risk with `data.costs(i) += ...`; they must not overwrite
  // costs already produced by Nav2's built-in critics.
  if (!clock_) {
    RCLCPP_ERROR(logger_, "DynamicRiskCritic has no ROS clock; skipping score");
    return;
  }

  const auto & shape = data.trajectories.x.shape();
  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 2000,
    "DynamicRiskCritic empty score: %zu candidate(s), %zu time step(s)",
    static_cast<std::size_t>(shape[0]), static_cast<std::size_t>(shape[1]));
}

}  // namespace mppi::critics

// Export the class symbol so pluginlib can construct it after it locates the
// matching XML declaration through the ament resource index.
PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicRiskCritic, mppi::critics::CriticFunction)
