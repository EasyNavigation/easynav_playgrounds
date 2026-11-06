cmake_minimum_required(VERSION 3.20)
project(easynav_playground_marine)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_files test/test_launch_files.py)
  ament_add_pytest_test(easynav_submarine test/test_easynav_submarine.py)
endif()

ament_package()
