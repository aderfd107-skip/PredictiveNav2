cmake_minimum_required(VERSION 3.8)
project(predictive_nav_tracking)

# ament_cmake provides ROS 2's CMake helpers, including ament_package().
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(predictive_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# The tracking node now receives cluster observations.  Later steps extend
# this same executable with track state, association and Kalman filtering.
add_executable(tracking_node src/tracking_node.cpp)
target_compile_features(tracking_node PUBLIC cxx_std_17)
ament_target_dependencies(tracking_node
  predictive_nav_msgs
  rclcpp
  visualization_msgs
)
target_link_libraries(tracking_node Eigen3::Eigen)

install(TARGETS
  tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
