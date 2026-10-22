#ifndef PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_
#define PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/subscription.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "predictive_nav_msgs/msg/predicted_trajectory_array.hpp"

// CriticManager in Nav2 Jazzy expands a YAML critic name (for example,
// "DynamicRiskCritic") to the plugin type "mppi::critics::DynamicRiskCritic".
// Therefore this custom critic follows Nav2's required plugin namespace even
// though its package and shared library are named predictive_nav_mppi.
namespace mppi::critics
{

/**
 * @brief Nav2 MPPI critic that caches and validates prediction messages.
 *
 * It deliberately does not change CriticData::costs yet. Dynamic-risk logic
 * is added only after the cached prediction passes frame, timestamp, and
 * trajectory-data validation.
 */
class DynamicRiskCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  using PredictionArray = predictive_nav_msgs::msg::PredictedTrajectoryArray;

  void prediction_callback(PredictionArray::ConstSharedPtr message);
  PredictionArray::ConstSharedPtr latest_prediction_snapshot() const;
  bool is_prediction_frame_usable(
    const PredictionArray & prediction, const rclcpp::Time & now,
    double * age_s, std::string * rejection_reason) const;
  bool is_predicted_trajectory_usable(
    const predictive_nav_msgs::msg::PredictedTrajectory & trajectory,
    std::string * rejection_reason) const;

  // Kept from initialize() so callbacks and score() can use ROS-time-aware
  // throttled logs.
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<PredictionArray>::SharedPtr prediction_subscription_;
  std::string prediction_topic_;
  std::string expected_prediction_frame_;
  double max_prediction_age_s_{0.0};
  double max_future_time_skew_s_{0.0};

  // The callback writes one shared pointer; score() copies that pointer and
  // releases the lock before doing any controller-loop work.
  mutable std::mutex prediction_mutex_;
  PredictionArray::ConstSharedPtr latest_prediction_;
};

}  // namespace mppi::critics

#endif  // PREDICTIVE_NAV_MPPI__DYNAMIC_RISK_CRITIC_HPP_
