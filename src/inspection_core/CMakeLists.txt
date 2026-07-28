cmake_minimum_required(VERSION 3.8)
project(inspection_core)

find_package(ament_cmake REQUIRED)

# Pure Python — nothing to build, only install.
install(
  DIRECTORY inspection_core
  DESTINATION lib/python3.12/site-packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
