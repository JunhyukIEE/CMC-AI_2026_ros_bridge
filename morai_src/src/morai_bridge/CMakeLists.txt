cmake_minimum_required(VERSION 3.8)
project(morai_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(morai_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Receiver Node
add_executable(morai_receiver_node src/morai_receiver_node.cpp)
target_include_directories(morai_receiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(morai_receiver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  morai_receiver_node
  "rclcpp"
  "morai_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
target_link_libraries(morai_receiver_node Threads::Threads)

# Sender Node
add_executable(morai_sender_node src/morai_sender_node.cpp)
target_include_directories(morai_sender_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(morai_sender_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  morai_sender_node
  "rclcpp"
  "morai_msgs"
)

install(TARGETS 
  morai_receiver_node
  morai_sender_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/morai_sensor_receiver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME morai_sensor_receiver_node
)

install(PROGRAMS
  scripts/navsat_to_gnss_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navsat_to_gnss_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
