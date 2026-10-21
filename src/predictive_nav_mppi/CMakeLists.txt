cmake_minimum_required(VERSION 3.8)
project(predictive_nav_mppi)

# This package exports an empty DynamicRiskCritic. Dynamic-risk scoring is added
# in later chapter steps, after plugin discovery/loading is verified.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(predictive_nav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dynamic_risk_critic.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_mppi_controller
  nav2_costmap_2d
  pluginlib
  predictive_nav_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Register the XML in the ament resource index and install it beside the
# package. Nav2's CriticManager discovers custom MPPI critics through this.
pluginlib_export_plugin_description_file(
  nav2_mppi_controller predictive_nav_mppi_critics.xml
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_mppi_controller
  nav2_costmap_2d
  pluginlib
  predictive_nav_msgs
)

ament_package()
