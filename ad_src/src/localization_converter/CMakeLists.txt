cmake_minimum_required(VERSION 3.8)
project(localization_converter)

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
find_package(morai_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add the library for the converter class
add_library(localization_converter_lib src/localization_converter.cpp)
target_include_directories(localization_converter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(localization_converter_lib
  rclcpp
  morai_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros)

# Add the executable for the node
add_executable(localization_converter_node src/localization_converter_node.cpp)
ament_target_dependencies(localization_converter_node rclcpp)
target_link_libraries(localization_converter_node localization_converter_lib)

# Install rules
install(
  TARGETS localization_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS localization_converter_lib
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
