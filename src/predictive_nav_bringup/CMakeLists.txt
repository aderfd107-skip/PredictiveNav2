cmake_minimum_required(VERSION 3.8)
project(predictive_nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
