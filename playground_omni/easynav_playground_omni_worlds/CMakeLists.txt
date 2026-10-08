# Copyright (c) 2025 Yohan Prakoso
# Copyright (c) 2026 Francisco Martín Rico
# SPDX-License-Identifier: MIT AND Apache-2.0

cmake_minimum_required(VERSION 3.20)
project(easynav_playground_omni_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(KINEMATICS_DEPENDENCIES
  rclcpp::rclcpp
  Eigen3::Eigen
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(kinematics src/kinematics.cpp)
target_link_libraries(kinematics PRIVATE ${KINEMATICS_DEPENDENCIES})
install(TARGETS kinematics DESTINATION lib/${PROJECT_NAME})

# install selected python code
# install(PROGRAMS scripts/subscriber.py
#         DESTINATION lib/${PROJECT_NAME})

# install all python code inside scripts folder and add exec permission
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  GROUP_EXECUTE GROUP_READ
  WORLD_EXECUTE WORLD_READ
  FILES_MATCHING PATTERN "*.py")

install(DIRECTORY
  launch
  config
  worlds
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  LICENSE
  LICENSE-APACHE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
