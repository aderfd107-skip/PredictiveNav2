#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace predictive_nav
{

// A valid laser return still needs its original beam index.  Step 05 will use
// that index together with angle_min and angle_increment to calculate (x, y).
struct ValidRange
{
  std::size_t beam_index{0U};
  float range_m{0.0F};
};

// Keeping each discard reason separate makes bad sensor data visible instead
// of silently disappearing from the algorithm.
struct RangeFilterResult
{
  std::size_t total_count{0U};
  std::size_t non_finite_count{0U};
  std::size_t out_of_range_count{0U};
  std::vector<ValidRange> valid_ranges;
};

struct CartesianPoint
{
  std::size_t beam_index{0U};
  double range_m{0.0};
  double angle_rad{0.0};
  double x_m{0.0};
  double y_m{0.0};
};

class ScanInfoNode : public rclcpp::Node
{
public:
  ScanInfoNode()
  : Node("scan_info_node")
  {
    min_detection_range_ = declare_parameter<double>("min_detection_range", 0.15);
    max_detection_range_ = declare_parameter<double>("max_detection_range", 6.00);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        scan_callback(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for LaserScan messages on /scan; keeping distances in [%.2f, %.2f] m.",
      min_detection_range_, max_detection_range_);
  }

private:
  RangeFilterResult filter_ranges(const sensor_msgs::msg::LaserScan & scan) const
  {
    RangeFilterResult result;
    result.total_count = scan.ranges.size();

    // The sensor message declares its physical limits.  The node parameters
    // apply the narrower range we want to use for this navigation project.
    const double effective_min_range = std::max(
      static_cast<double>(scan.range_min), min_detection_range_);
    const double effective_max_range = std::min(
      static_cast<double>(scan.range_max), max_detection_range_);

    for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
      const float range = scan.ranges[index];

      if (!std::isfinite(range)) {
        ++result.non_finite_count;
        continue;
      }

      if (
        range < static_cast<float>(effective_min_range) ||
        range > static_cast<float>(effective_max_range))
      {
        ++result.out_of_range_count;
        continue;
      }

      result.valid_ranges.push_back(ValidRange{index, range});
    }

    return result;
  }

  std::vector<CartesianPoint> ranges_to_lidar_points(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ValidRange> & valid_ranges) const
  {
    std::vector<CartesianPoint> points;
    points.reserve(valid_ranges.size());

    for (const ValidRange & valid_range : valid_ranges) {
      const double angle_rad = static_cast<double>(scan.angle_min) +
        static_cast<double>(valid_range.beam_index) *
        static_cast<double>(scan.angle_increment);
      const double range_m = static_cast<double>(valid_range.range_m);
      const double x_m = range_m * std::cos(angle_rad);
      const double y_m = range_m * std::sin(angle_rad);

      points.push_back(
        CartesianPoint{
          valid_range.beam_index,
          range_m,
          angle_rad,
          x_m,
          y_m
        });
    }

    return points;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    ++message_count_;
    const RangeFilterResult filter_result = filter_ranges(*message);
    const std::vector<CartesianPoint> lidar_points = ranges_to_lidar_points(*message, filter_result.valid_ranges);

    // A LiDAR can publish many times per second.  Logging every tenth message
    // keeps the terminal readable while still proving that data keeps arriving.
    if (message_count_ % 10U != 0U) {
      return;
    }

    if (!lidar_points.empty()) {
      const CartesianPoint & first_point = lidar_points.front();
      RCLCPP_INFO(
        get_logger(),
        "first valid point | beam=%zu | range=%.2f m | angle=%.3f rad | "
        "lidar_link=(%.2f, %.2f) m",
        first_point.beam_index,
        first_point.range_m,
        first_point.angle_rad,
        first_point.x_m,
        first_point.y_m);
    }



    RCLCPP_INFO(
      get_logger(),
      "scan #%zu | frame=%s | stamp=%d.%09u | ranges=%zu | "
      "kept=%zu | discarded(non_finite=%zu, out_of_range=%zu) | "
      "lidar_points=%zu | angle_min=%.3f rad | angle_increment=%.5f rad | "
      "filter_range=[%.2f, %.2f] m",
      message_count_,
      message->header.frame_id.c_str(),
      static_cast<int>(message->header.stamp.sec),
      static_cast<unsigned int>(message->header.stamp.nanosec),
      message->ranges.size(),
      filter_result.valid_ranges.size(),
      filter_result.non_finite_count,
      filter_result.out_of_range_count,
      lidar_points.size(),
      static_cast<double>(message->angle_min),
      static_cast<double>(message->angle_increment),
      min_detection_range_,
      max_detection_range_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::size_t message_count_{0U};
  double min_detection_range_{0.15};
  double max_detection_range_{6.00};
};

}  // namespace predictive_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<predictive_nav::ScanInfoNode>());
  rclcpp::shutdown();
  return 0;
}
