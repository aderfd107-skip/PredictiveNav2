cmake_minimum_required(VERSION 3.8)
project(predictive_nav_perception)

# ament_cmake is ROS 2's CMake extension.  It provides ament_package().
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(predictive_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# The first real node: subscribes to /scan and prints one summary every ten
# messages.  Later steps will extend this package with perception algorithms.
add_executable(scan_info_node src/scan_info_node.cpp)
target_compile_features(scan_info_node PUBLIC cxx_std_17)
ament_target_dependencies(scan_info_node
  geometry_msgs
  predictive_nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  scan_info_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
