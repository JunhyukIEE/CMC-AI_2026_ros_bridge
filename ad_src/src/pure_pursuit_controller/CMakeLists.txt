cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

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
find_package(morai_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the library for the controller class
add_library(pure_pursuit_controller_lib src/pure_pursuit_controller.cpp)
target_include_directories(pure_pursuit_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pure_pursuit_controller_lib
  rclcpp
  nav_msgs
  morai_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs)

# Add the executable for the node
add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp)
target_link_libraries(pure_pursuit_node
  pure_pursuit_controller_lib)

# Install rules
install(
  TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS pure_pursuit_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES launch/pure_pursuit.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  FILES config/params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
