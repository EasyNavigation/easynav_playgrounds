cmake_minimum_required(VERSION 3.20)
project(easynav_playground_geo_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/geo_world.py DESTINATION lib/${PROJECT_NAME})
install(FILES tools/tree_models.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # tools/ (geo_world.py, tree_models.py) come from a code base that was never held to the style
  # linters: the launch files and the tests are what is checked
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257 ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  # the decisions of the simulation launch file (no process is started)
  ament_add_pytest_test(geo_sim_launch test/test_geo_sim_launch.py)
  # the world generator: what works offline (no download)
  ament_add_pytest_test(geo_world test/test_geo_world.py)
endif()

ament_package()
