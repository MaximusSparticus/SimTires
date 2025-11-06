cmake_minimum_required(VERSION 3.20)
project(SimTires
    VERSION 0.1.0
    DESCRIPTION "Simulation environment for terrain challenged autonomous vehicle R&D built on Project Chrono - namesake is the SpinTires video game franchise"
    LANGUAGES CXX
)

# Setup Output Directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")

# Dependencies
include(cmake/ChronoTargets.cmake)
# TODO(zmd): ROS2 dependencies
find_package(Eigen3 REQUIRED)

# SimTires Executable
add_executable(SimTires
    ${CMAKE_CURRENT_SOURCE_DIR}/src/exec/main.cpp
)

target_include_directories(SimTires
    PRIVATE
        # TODO(zmd): ROS2 includes
        ${CMAKE_CURRENT_SOURCE_DIR}/src/
        ${CHRONO_INCLUDE_DIRS}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(SimTires PRIVATE
    Chrono::Engine
    Chrono::Vehicle
    # TODO(zmd): Chrono::ROS
    # TODO(zmd): Chrono::Sensor
    # TODO(zmd): Chrono::Parsers
    Chrono::Irrlicht
    Chrono::Vehicle_Irrlicht
    Eigen3::Eigen
)
