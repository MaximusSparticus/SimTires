cmake_minimum_required(VERSION 3.5)
project(teleop_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(chrono_ros_interface REQUIRED)

add_executable(teleop_driver src/teleop_driver.cpp)
ament_target_dependencies(teleop_driver rclcpp std_msgs sensor_msgs joy chrono_ros_interface)

install(TARGETS
  teleop_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
