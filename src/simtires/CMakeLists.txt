cmake_minimum_required(VERSION 3.20)
project(simtires)

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

# Find Chrono ROS interface
find_package(chrono_ros_interfaces REQUIRED)

# Add the chrono library to module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../install/chrono_wrapper/chrono/lib/cmake/Chrono")
set(OptiX_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/optix")

# Find all Chrono dependencies first so the targets exist
find_package(urdfdom REQUIRED)
find_package(Irrlicht REQUIRED)
find_package(OptiX REQUIRED)

# Find chrono and chrono_wrapper - TODO(zmd): do we need to find chrono this way? Seems convoluted
find_package(chrono_wrapper REQUIRED)
find_package(Chrono REQUIRED 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../install/chrono_wrapper/chrono/lib/cmake/Chrono
    NO_DEFAULT_PATH
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# SimTires simulation node
add_executable(simulation_node
    src/simulation_node.cpp
    src/simulation_world.cpp
    src/vehicle_manager.cpp
    src/sensor_manager.cpp
    src/ros_interface.cpp
    src/visualization_manager.cpp
    src/chrono_solver_setup.cpp
)

# ROS2 dependencies for the simulation node
ament_target_dependencies(simulation_node
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

# Link Chrono libraries
target_link_libraries(simulation_node 
    Chrono::Chrono_core
    Chrono::Chrono_vehicle
    Chrono::Chrono_vehicle_irrlicht
    Chrono::Chrono_irrlicht
    Chrono::Chrono_ros
    Chrono::Chrono_sensor
    Chrono::Chrono_parsers
    Eigen3::Eigen
)

# Set RPATH so simulation_node can find Chrono libraries at runtime
set_target_properties(simulation_node PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../install/chrono_wrapper/chrono/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Set Chrono data directory
get_filename_component(CHRONO_DATA_DIR_ABS 
    "${CMAKE_CURRENT_SOURCE_DIR}/../../install/chrono_wrapper/chrono/share/chrono/data/" 
    ABSOLUTE
)
target_compile_definitions(simulation_node PRIVATE
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR_ABS}"
)
message(STATUS "Setting CHRONO_DATA_DIR to: ${CHRONO_DATA_DIR_ABS}")

# Install the executable
install(TARGETS simulation_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (TODO(zmd): add config files?
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config/)
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
endif()

# Install include headers
install(DIRECTORY include/
    DESTINATION include/
)

ament_package()
