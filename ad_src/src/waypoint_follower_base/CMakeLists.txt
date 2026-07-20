cmake_minimum_required(VERSION 3.8)
project(waypoint_follower_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(morai_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Build trajectory_manager library
add_library(trajectory_manager SHARED
  src/trajectory_manager.cpp
)
ament_target_dependencies(trajectory_manager
  rclcpp
)

# Build waypoint_follower_base library
add_library(waypoint_follower_base_lib SHARED
  src/waypoint_follower_base.cpp
)
ament_target_dependencies(waypoint_follower_base_lib
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  morai_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(waypoint_follower_base_lib
  trajectory_manager
)

# Build waypoint_follower_base_node executable
add_executable(waypoint_follower_base_node
  src/waypoint_follower_base_node.cpp
)
ament_target_dependencies(waypoint_follower_base_node
  rclcpp
)
target_link_libraries(waypoint_follower_base_node
  waypoint_follower_base_lib
  trajectory_manager
)

# Install targets
install(TARGETS
  trajectory_manager
  waypoint_follower_base_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  waypoint_follower_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
