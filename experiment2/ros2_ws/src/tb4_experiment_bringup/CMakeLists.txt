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
  PROGRAMS scripts/validate_vmware_simulation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_vmware_simulation
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
