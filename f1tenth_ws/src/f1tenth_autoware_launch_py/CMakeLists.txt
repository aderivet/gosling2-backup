cmake_minimum_required(VERSION 3.14)
project(f1tenth_autoware_launch_py)

find_package(ament_cmake_auto REQUIRED)

# TODO: remove temporary workaround for states
install(
  PROGRAMS scripts/init_state.sh scripts/scenario.sh scripts/localization_trigger.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  rviz
  config
)
