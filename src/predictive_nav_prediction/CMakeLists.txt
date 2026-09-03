cmake_minimum_required(VERSION 3.8)
project(predictive_nav_prediction)

# ament_cmake provides ROS 2's CMake helpers, including ament_package().
find_package(ament_cmake REQUIRED)
find_package(predictive_nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Step 04 reads the tracking module's public message.  Prediction algorithms
# and publishers will be added in later steps.
add_executable(prediction_node src/prediction_node.cpp)
target_compile_features(prediction_node PUBLIC cxx_std_17)
ament_target_dependencies(prediction_node
  predictive_nav_msgs
  rclcpp
)

install(TARGETS
  prediction_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
