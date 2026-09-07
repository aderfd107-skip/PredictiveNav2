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

    if (!std::isfinite(max_track_message_age_s_) || max_track_message_age_s_ <= 0.0 ||
      !std::isfinite(max_future_time_skew_s_) || max_future_time_skew_s_ < 0.0 ||
      !std::isfinite(min_track_confidence_) || min_track_confidence_ < 0.0 ||
      min_track_confidence_ > 1.0 || !std::isfinite(max_track_speed_mps_) ||
      max_track_speed_mps_ <= 0.0)
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
      "prediction_node started. Step 09 publishes accepted trajectories on "
      "/dynamic_obstacles/predictions and never controls Nav2.");
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
