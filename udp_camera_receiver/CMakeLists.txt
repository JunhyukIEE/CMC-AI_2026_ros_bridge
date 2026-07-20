cmake_minimum_required(VERSION 3.5)
project(udp_camera_receiver)

# Default to C++17 for better std::optional support
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Find CUDA and nvJPEG
find_package(CUDA REQUIRED)
find_library(NVJPEG_LIBRARY nvjpeg HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}_lib
  src/udp_camera_receiver.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
  ${NVJPEG_LIBRARY}
  ${CUDA_LIBRARIES}
)

# Create executable
add_executable(${PROJECT_NAME}_node
  src/udp_camera_receiver_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
