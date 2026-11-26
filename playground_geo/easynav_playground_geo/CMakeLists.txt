cmake_minimum_required(VERSION 3.20)
project(easynav_playground_geo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # the launch file against the launch file of the simulation it forwards to (no process starts)
  ament_add_pytest_test(launch_files test/test_launch_files.py)
endif()

ament_package()
