cmake_minimum_required(VERSION 3.5)

project(dienen_behaviors)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(keisan REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tosshin_cpp REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_library(${PROJECT_NAME}
  "src/patrol_behavior.cpp"
  "src/teleop_behavior.cpp")

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs keisan nav_msgs rclcpp sensor_msgs tf2_msgs tosshin_cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

set(EXECUTABLES
  "bin/patrol_behavior.cpp"
  "bin/slam_bridge.cpp"
  "bin/teleop_behavior.cpp")

foreach(EXECUTABLE ${EXECUTABLES})
  get_filename_component(TARGET ${EXECUTABLE} NAME_WE)

  add_executable(${TARGET} ${EXECUTABLE})
  target_link_libraries(${TARGET} ${PROJECT_NAME})

  install(TARGETS ${TARGET}
    DESTINATION "lib/${PROJECT_NAME}")
endforeach()

install(DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  geometry_msgs keisan nav_msgs rclcpp sensor_msgs tf2_msgs tosshin_cpp)

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
