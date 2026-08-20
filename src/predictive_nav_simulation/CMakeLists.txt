cmake_minimum_required(VERSION 3.8)
project(predictive_nav_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dynamic_obstacle_controller.py
    scripts/scan_safety_guard.py
    scripts/static_world_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
