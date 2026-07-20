cmake_minimum_required(VERSION 3.8)
project(waypoint_recorder)

# Default to C++17 (required for ROS2 Humble)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the library for the recorder class
add_library(waypoint_recorder_lib src/waypoint_recorder.cpp)
target_include_directories(waypoint_recorder_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(waypoint_recorder_lib
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs)

# Add the executable for the node
add_executable(waypoint_recorder_node src/waypoint_recorder_node.cpp)
ament_target_dependencies(waypoint_recorder_node rclcpp)
target_link_libraries(waypoint_recorder_node waypoint_recorder_lib)

# Install rules
install(
  TARGETS waypoint_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS waypoint_recorder_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
