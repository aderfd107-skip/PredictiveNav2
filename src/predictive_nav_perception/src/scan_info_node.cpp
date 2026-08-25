#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "predictive_nav_msgs/msg/obstacle_cluster_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

// This point has the same physical location as a CartesianPoint, but its x/y
// coordinates are expressed in the stable tracking frame (normally odom).
struct TrackingPoint
{
  std::size_t beam_index{0U};
  double range_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
};

// A cluster is one spatially connected group of scan points.  At this stage it
// is only a geometric observation: it may be a wall segment, furniture, or a
// moving actor.  Tracking across multiple frames will decide what is dynamic.
struct ObstacleCluster
{
  std::size_t point_count{0U};
  double centroid_x_m{0.0};
  double centroid_y_m{0.0};
  double size_x_m{0.0};
  double size_y_m{0.0};
};

struct ClusteringResult
{
  std::vector<ObstacleCluster> clusters;
  std::size_t rejected_too_small{0U};
  std::size_t rejected_too_large{0U};
};

class ScanInfoNode : public rclcpp::Node
{
public:
  ScanInfoNode()
  : Node("scan_info_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, false)
  {
    min_detection_range_ = declare_parameter<double>("min_detection_range", 0.15);
    max_detection_range_ = declare_parameter<double>("max_detection_range", 6.00);
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "odom");
    cluster_distance_threshold_m_ = declare_parameter<double>(
      "cluster_distance_threshold_m", 0.18);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 100);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        scan_callback(message);
      });

    cluster_publisher_ = create_publisher<predictive_nav_msgs::msg::ObstacleClusterArray>(
      "/dynamic_obstacles/clusters", rclcpp::SensorDataQoS());
    cluster_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dynamic_obstacles/cluster_markers", 10);

    RCLCPP_INFO(
      get_logger(),
      "Waiting for LaserScan messages on /scan; keeping distances in [%.2f, %.2f] m; "
      "transforming points into %s; clustering adjacent points within %.2f m.",
      min_detection_range_, max_detection_range_, tracking_frame_.c_str(),
      cluster_distance_threshold_m_);
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

  std::vector<TrackingPoint> transform_to_tracking_frame(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<CartesianPoint> & lidar_points)
  {
    const auto transform = tf_buffer_.lookupTransform(
      tracking_frame_,
      scan.header.frame_id,
      rclcpp::Time(scan.header.stamp));

    std::vector<TrackingPoint> transformed_points;
    transformed_points.reserve(lidar_points.size());

    geometry_msgs::msg::PointStamped lidar_point_message;
    lidar_point_message.header = scan.header;
    lidar_point_message.point.z = 0.0;

    for (const CartesianPoint & lidar_point : lidar_points) {
      lidar_point_message.point.x = lidar_point.x_m;
      lidar_point_message.point.y = lidar_point.y_m;

      //真正的把scan坐标转换成odom坐标
      geometry_msgs::msg::PointStamped tracking_point_message;
      tf2::doTransform(lidar_point_message, tracking_point_message, transform);

      transformed_points.push_back(TrackingPoint{
        lidar_point.beam_index,
        lidar_point.range_m,
        tracking_point_message.point.x,
        tracking_point_message.point.y});
    }

    return transformed_points;
  }

  bool are_neighbouring_scan_points(
    const TrackingPoint & previous,
    const TrackingPoint & current) const
  {
    // A gap in original beam indices means an invalid/missing beam occurred,
    // so points on either side must not be silently joined into one object.
    if (current.beam_index != previous.beam_index + 1U) {
      return false;
    }

    const double distance_m = std::hypot(
      current.x_m - previous.x_m,
      current.y_m - previous.y_m);
    return distance_m <= cluster_distance_threshold_m_;
  }

  void append_cluster_if_valid(
    const std::vector<TrackingPoint> & candidate_points,
    ClusteringResult & result) const
  {
    if (candidate_points.empty()) {
      return;
    }

    const std::size_t point_count = candidate_points.size();
    if (point_count < static_cast<std::size_t>(min_cluster_points_)) {
      ++result.rejected_too_small;
      return;
    }
    if (point_count > static_cast<std::size_t>(max_cluster_points_)) {
      ++result.rejected_too_large;
      return;
    }

    double sum_x_m = 0.0;
    double sum_y_m = 0.0;
    double min_x_m = std::numeric_limits<double>::infinity();
    double max_x_m = -std::numeric_limits<double>::infinity();
    double min_y_m = std::numeric_limits<double>::infinity();
    double max_y_m = -std::numeric_limits<double>::infinity();

    for (const TrackingPoint & point : candidate_points) {
      sum_x_m += point.x_m;
      sum_y_m += point.y_m;
      min_x_m = std::min(min_x_m, point.x_m);
      max_x_m = std::max(max_x_m, point.x_m);
      min_y_m = std::min(min_y_m, point.y_m);
      max_y_m = std::max(max_y_m, point.y_m);
    }

    result.clusters.push_back(ObstacleCluster{
      point_count,
      sum_x_m / static_cast<double>(point_count),
      sum_y_m / static_cast<double>(point_count),
      max_x_m - min_x_m,
      max_y_m - min_y_m});
  }

  ClusteringResult cluster_tracking_points(
    const std::vector<TrackingPoint> & tracking_points) const
  {
    ClusteringResult result;
    std::vector<TrackingPoint> candidate_points;
    candidate_points.reserve(tracking_points.size());

    for (const TrackingPoint & point : tracking_points) {
      if (
        !candidate_points.empty() &&
        !are_neighbouring_scan_points(candidate_points.back(), point))
      {
        append_cluster_if_valid(candidate_points, result);
        candidate_points.clear();
      }

      candidate_points.push_back(point);
    }

    append_cluster_if_valid(candidate_points, result);
    return result;
  }

  void publish_clusters(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ObstacleCluster> & clusters)
  {
    predictive_nav_msgs::msg::ObstacleClusterArray observation_message;

    // The centroids were transformed to tracking_frame_, so the outgoing
    // header must not keep scan.header.frame_id (normally lidar_link).
    observation_message.header.stamp = scan.header.stamp;
    observation_message.header.frame_id = tracking_frame_;
    observation_message.clusters.reserve(clusters.size());

    for (const ObstacleCluster & cluster : clusters) {
      predictive_nav_msgs::msg::ObstacleCluster cluster_message;
      cluster_message.centroid.x = cluster.centroid_x_m;
      cluster_message.centroid.y = cluster.centroid_y_m;
      cluster_message.centroid.z = 0.0;
      cluster_message.size_x_m = static_cast<float>(cluster.size_x_m);
      cluster_message.size_y_m = static_cast<float>(cluster.size_y_m);
      cluster_message.point_count = static_cast<std::uint32_t>(cluster.point_count);
      observation_message.clusters.push_back(cluster_message);
    }

    cluster_publisher_->publish(observation_message);
  }

  void publish_cluster_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<ObstacleCluster> & clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // Cluster count can shrink between frames.  Clearing this topic first
    // prevents a box for an old cluster from remaining visible in RViz.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for (std::size_t index = 0U; index < clusters.size(); ++index) {
      const ObstacleCluster & cluster = clusters[index];
      const int marker_id = static_cast<int>(index);

      visualization_msgs::msg::Marker box_marker;
      box_marker.header.stamp = scan.header.stamp;
      box_marker.header.frame_id = tracking_frame_;
      box_marker.ns = "scan_derived_cluster_boxes";
      box_marker.id = marker_id;
      box_marker.type = visualization_msgs::msg::Marker::CUBE;
      box_marker.action = visualization_msgs::msg::Marker::ADD;
      box_marker.pose.position.x = cluster.centroid_x_m;
      box_marker.pose.position.y = cluster.centroid_y_m;
      box_marker.pose.position.z = 0.05;
      box_marker.pose.orientation.w = 1.0;
      // A very thin cluster should remain visible instead of becoming a zero-
      // width box.  The stored message values remain the unmodified sizes.
      box_marker.scale.x = std::max(cluster.size_x_m, 0.05);
      box_marker.scale.y = std::max(cluster.size_y_m, 0.05);
      box_marker.scale.z = 0.10;
      box_marker.color.r = 0.0F;
      box_marker.color.g = 0.90F;
      box_marker.color.b = 1.0F;
      box_marker.color.a = 0.35F;
      marker_array.markers.push_back(box_marker);

      visualization_msgs::msg::Marker centroid_marker;
      centroid_marker.header = box_marker.header;
      centroid_marker.ns = "scan_derived_cluster_centroids";
      centroid_marker.id = marker_id;
      centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::msg::Marker::ADD;
      centroid_marker.pose.position.x = cluster.centroid_x_m;
      centroid_marker.pose.position.y = cluster.centroid_y_m;
      centroid_marker.pose.position.z = 0.12;
      centroid_marker.pose.orientation.w = 1.0;
      centroid_marker.scale.x = 0.12;
      centroid_marker.scale.y = 0.12;
      centroid_marker.scale.z = 0.12;
      centroid_marker.color.r = 1.0F;
      centroid_marker.color.g = 0.15F;
      centroid_marker.color.b = 0.0F;
      centroid_marker.color.a = 0.95F;
      marker_array.markers.push_back(centroid_marker);
    }

    cluster_marker_publisher_->publish(marker_array);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    ++message_count_;
    const RangeFilterResult filter_result = filter_ranges(*message);
    const std::vector<CartesianPoint> lidar_points = ranges_to_lidar_points(
      *message, filter_result.valid_ranges);

    std::vector<TrackingPoint> tracking_points;
    try {
      tracking_points = transform_to_tracking_frame(*message, lidar_points);
    } catch (const tf2::TransformException & exception) {
      ++tf_failure_count_;
      if (message_count_ % 10U == 0U) {
        RCLCPP_WARN(
          get_logger(),
          "Skipping scan because transform %s <- %s at the scan timestamp is unavailable: %s",
          tracking_frame_.c_str(),
          message->header.frame_id.c_str(),
          exception.what());
      }
      return;
    }
    const ClusteringResult clustering_result = cluster_tracking_points(tracking_points);
    publish_clusters(*message, clustering_result.clusters);
    publish_cluster_markers(*message, clustering_result.clusters);

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

    if (!tracking_points.empty()) {
      const TrackingPoint & first_point = tracking_points.front();
      RCLCPP_INFO(
        get_logger(),
        "first transformed point | beam=%zu | range=%.2f m | %s=(%.2f, %.2f) m",
        first_point.beam_index,
        first_point.range_m,
        tracking_frame_.c_str(),
        first_point.x_m,
        first_point.y_m);
    }

    if (!clustering_result.clusters.empty()) {
      const ObstacleCluster & first_cluster = clustering_result.clusters.front();
      RCLCPP_INFO(
        get_logger(),
        "first cluster | points=%zu | centroid_%s=(%.2f, %.2f) m | size=(%.2f, %.2f) m",
        first_cluster.point_count,
        tracking_frame_.c_str(),
        first_cluster.centroid_x_m,
        first_cluster.centroid_y_m,
        first_cluster.size_x_m,
        first_cluster.size_y_m);
    }



    RCLCPP_INFO(
      get_logger(),
      "scan #%zu | frame=%s | stamp=%d.%09u | ranges=%zu | "
      "kept=%zu | discarded(non_finite=%zu, out_of_range=%zu) | "
      "lidar_points=%zu | tracking_points=%zu | tf_failures_total=%zu | "
      "clusters=%zu | rejected_clusters(small=%zu, large=%zu) | "
      "angle_min=%.3f rad | angle_increment=%.5f rad | "
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
      tracking_points.size(),
      tf_failure_count_,
      clustering_result.clusters.size(),
      clustering_result.rejected_too_small,
      clustering_result.rejected_too_large,
      static_cast<double>(message->angle_min),
      static_cast<double>(message->angle_increment),
      min_detection_range_,
      max_detection_range_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<predictive_nav_msgs::msg::ObstacleClusterArray>::SharedPtr cluster_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::size_t message_count_{0U};
  std::size_t tf_failure_count_{0U};
  double min_detection_range_{0.15};
  double max_detection_range_{6.00};
  std::string tracking_frame_{"odom"};
  double cluster_distance_threshold_m_{0.18};
  int min_cluster_points_{3};
  int max_cluster_points_{100};
};

}  // namespace predictive_nav

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<predictive_nav::ScanInfoNode>());
  rclcpp::shutdown();
  return 0;
}
