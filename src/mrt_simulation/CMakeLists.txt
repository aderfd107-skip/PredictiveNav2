cmake_minimum_required(VERSION 3.8)
project(mrt_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/collision_guard.py
    scripts/office_map_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
