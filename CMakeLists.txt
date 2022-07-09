cmake_minimum_required(VERSION 3.5)
project(ros2-filters)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)

# Build
include_directories(include)

add_library(${PROJECT_NAME}
  SHARED
  src/test_filter.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  pluginlib
)

add_executable(pc_to_pc_filter_chain
  src/point_cloud_to_point_cloud_filter_chain.cpp
  src/point_cloud_to_point_cloud_filter_chain_node.cpp
)
ament_target_dependencies(pc_to_pc_filter_chain
  rclcpp
  sensor_msgs
  pluginlib
  message_filters
)

# Install
pluginlib_export_plugin_description_file(filters filters.xml)

install(DIRECTORY
  include/
  DESTINATION include
)

# ROS2 filters library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# point cloud to point cloud filter chain
install(TARGETS
  pc_to_pc_filter_chain
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  test
  rviz2
  DESTINATION share/${PROJECT_NAME}
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
