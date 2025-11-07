cmake_minimum_required(VERSION 3.20)
project(simtires_autonomy)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find chrono_ros_interfaces (for communication with SimTires simulation)
find_package(chrono_ros_interfaces REQUIRED)

# Include directories
include_directories(include)

# Autonomous driver node
add_executable(autonomous_driver_node
    src/autonomous_driver_node.cpp
)

# ROS2 dependencies for the autonomous driver
ament_target_dependencies(autonomous_driver_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    visualization_msgs
    chrono_ros_interfaces
)

# Install the executable
install(TARGETS autonomous_driver_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install include headers (if any)
install(DIRECTORY include/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
