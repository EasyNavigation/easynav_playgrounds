cmake_minimum_required(VERSION 3.20)
project(easynav_playground_aerial_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  models
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/ufo_flight_controller.py DESTINATION lib/${PROJECT_NAME})

install(FILES LICENSE LICENSE-CC0 LICENSE-CC-BY-4.0 DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
