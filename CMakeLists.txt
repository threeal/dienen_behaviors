cmake_minimum_required(VERSION 3.5)

project(dienen_behaviors)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
  argparse geometry_msgs keisan nav_msgs rclcpp rosgraph_msgs
  sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tosshin_cpp)

foreach(DEPENDENCY ${DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(EXECUTABLES
  "bin/move_for.cpp"
  "bin/odometry_echo.cpp"
  "bin/patrol_behavior.cpp"
  "bin/slam_bridge.cpp")

foreach(EXECUTABLE ${EXECUTABLES})
  get_filename_component(TARGET ${EXECUTABLE} NAME_WE)

  add_executable(${TARGET} ${EXECUTABLE})
  ament_target_dependencies(${TARGET} ${DEPENDENCIES})

  install(TARGETS ${TARGET}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

install(DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${DEPENDENCIES})

ament_package()
