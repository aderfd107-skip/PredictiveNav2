#ifndef PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_
#define PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_

#include <memory>

#include "rclcpp/clock.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

// CriticManager in Nav2 Jazzy expands a YAML critic name (for example,
// "DynamicRiskCritic") to the plugin type "mppi::critics::DynamicRiskCritic".
// Therefore this custom critic follows Nav2's required plugin namespace even
// though its package and shared library are named predictive_nav_mppi.
namespace mppi::critics
{

/**
 * @brief Empty Nav2 MPPI critic used to validate the plugin interface.
 *
 * This step deliberately does not subscribe to predictions or change
 * CriticData::costs. Dynamic-risk logic is added in later chapter steps.
 */
class DynamicRiskCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  // Kept from initialize() so score() can use ROS-time-aware throttled logs.
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace mppi::critics

#endif  // PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_
