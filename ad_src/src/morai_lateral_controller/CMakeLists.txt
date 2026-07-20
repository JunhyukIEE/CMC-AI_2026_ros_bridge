cmake_minimum_required(VERSION 3.8)
project(morai_lateral_controller)

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
find_package(morai_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find OSQP
find_library(OSQP_LIB osqp PATHS /usr/local/lib)
find_path(OSQP_INCLUDE_DIR osqp/osqp.h PATHS /usr/local/include)

if(NOT OSQP_LIB OR NOT OSQP_INCLUDE_DIR)
  message(WARNING "OSQP not found. MPC controller will not be built.")
  set(BUILD_MPC FALSE)
else()
  message(STATUS "Found OSQP: ${OSQP_LIB}")
  set(BUILD_MPC TRUE)
endif()

# Include directories
include_directories(
  include
  ${OSQP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# Build Pure Pursuit controller library
add_library(pure_pursuit_controller_lib SHARED
  src/pure_pursuit_controller.cpp
)
ament_target_dependencies(pure_pursuit_controller_lib
  rclcpp
  std_msgs
  morai_msgs
)

# Build Pure Pursuit controller node executable
add_executable(pure_pursuit_controller_node
  src/pure_pursuit_controller_node.cpp
)
ament_target_dependencies(pure_pursuit_controller_node
  rclcpp
)
target_link_libraries(pure_pursuit_controller_node
  pure_pursuit_controller_lib
)

# Build MPC controller (if OSQP is available)
if(BUILD_MPC)
  # Build MPC controller library
  add_library(mpc_controller_lib SHARED
    src/mpc_controller.cpp
  )
  ament_target_dependencies(mpc_controller_lib
    rclcpp
    std_msgs
    nav_msgs
    morai_msgs
    tf2_geometry_msgs
    visualization_msgs
  )
  target_link_libraries(mpc_controller_lib
    ${OSQP_LIB}
  )

  # Build MPC controller node executable
  add_executable(mpc_controller_node
    src/mpc_controller_node.cpp
  )
  ament_target_dependencies(mpc_controller_node
    rclcpp
  )
  target_link_libraries(mpc_controller_node
    mpc_controller_lib
  )

  # Install MPC targets
  install(TARGETS
    mpc_controller_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS
    mpc_controller_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install targets
install(TARGETS
  pure_pursuit_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pure_pursuit_controller_node
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
