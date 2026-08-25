cmake_minimum_required(VERSION 3.8)
project(predictive_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# A ROS 2 interface package contains the shared language-independent message
# definitions.  rosidl generates the C++ headers used by perception now and
# by the tracking package later.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleCluster.msg"
  "msg/ObstacleClusterArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
