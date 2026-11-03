cmake_minimum_required(VERSION 3.20)
project(easynav_playground_marine_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
