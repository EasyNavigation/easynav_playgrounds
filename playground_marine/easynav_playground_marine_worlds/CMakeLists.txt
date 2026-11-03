cmake_minimum_required(VERSION 3.20)
project(easynav_playground_marine_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/synthetic_sea.py tools/odom_to_tf.py tools/auv_cmd.py tools/boat_cmd.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the Python of tools/ was never held to the style linters: the launch files and the tests are
  # what is checked
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8 ament_cmake_pep257 ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(auv_cmd test/test_auv_cmd.py)
  ament_add_pytest_test(boat_cmd test/test_boat_cmd.py)
  ament_add_pytest_test(synthetic_sea test/test_synthetic_sea.py)
  ament_add_pytest_test(marine_sim_launch test/test_marine_sim_launch.py)
endif()

ament_package()
