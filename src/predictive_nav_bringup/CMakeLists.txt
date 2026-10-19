cmake_minimum_required(VERSION 3.8)
project(predictive_nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/publish_initial_pose.py
    scripts/corner_alignment_executor.py
    scripts/corner_alignment_goal_bridge.py
    scripts/dwb_evaluation_monitor.py
    scripts/narrow_passage_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
