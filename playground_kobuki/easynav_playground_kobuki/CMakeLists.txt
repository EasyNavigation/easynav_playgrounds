cmake_minimum_required(VERSION 3.20)
project(easynav_playground_kobuki)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
