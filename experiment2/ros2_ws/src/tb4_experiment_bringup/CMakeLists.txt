cmake_minimum_required(VERSION 3.8)
project(tb4_experiment_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/vmware_simulation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_extended_mapping_route.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_extended_mapping_route
)

install(
  PROGRAMS scripts/run_long_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_long_navigation
)

install(
  PROGRAMS scripts/validate_localization_stability.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_localization_stability
)

install(
  PROGRAMS scripts/validate_slam_coverage.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_slam_coverage
)

install(
  PROGRAMS scripts/validate_vmware_simulation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_vmware_simulation
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
    test_simulation_launch
    test/test_simulation_launch.py
  )
endif()

ament_package()
