cmake_minimum_required(VERSION 3.8)
project(wro_sim)

find_package(ament_cmake REQUIRED)

# Install runtime data.
install(DIRECTORY
    worlds
    models
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Environment hook: prepend our worlds/ and models/ to GZ_SIM_RESOURCE_PATH so
# gz-sim can find the robot model and world files at runtime.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hook/wro_sim.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
