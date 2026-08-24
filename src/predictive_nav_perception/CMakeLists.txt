cmake_minimum_required(VERSION 3.8)
project(predictive_nav_perception)

# ament_cmake is ROS 2's CMake extension.  It provides ament_package().
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# The first real node: subscribes to /scan and prints one summary every ten
# messages.  Later steps will extend this package with perception algorithms.
add_executable(scan_info_node src/scan_info_node.cpp)
target_compile_features(scan_info_node PUBLIC cxx_std_17)
ament_target_dependencies(scan_info_node rclcpp sensor_msgs)

install(TARGETS
  scan_info_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
