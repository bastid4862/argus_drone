cmake_minimum_required(VERSION 3.8)
project(scan_head_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cave_drone_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

# Build the scan head bridge node
add_executable(
        scan_head_bridge_node
        src/scan_head_bridge_node.cpp
)

# Link the ROS 2 dependencies used by this node
ament_target_dependencies(
        scan_head_bridge_node
        rclcpp
        std_msgs
        std_srvs
        cave_drone_interfaces
)

# Install the executable so ros2 run can find it
install(
        TARGETS scan_head_bridge_node
        DESTINATION lib/${PROJECT_NAME}
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
