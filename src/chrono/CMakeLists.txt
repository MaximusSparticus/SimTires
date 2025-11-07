cmake_minimum_required(VERSION 3.20)
project(chrono)

# Find ament cmake for ROS2
find_package(ament_cmake REQUIRED)

# CRITICAL: Find chrono_ros_interfaces first
find_package(chrono_ros_interfaces REQUIRED)

include(ExternalProject)

# Get install prefix from ament
if(NOT DEFINED CMAKE_INSTALL_PREFIX OR CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/install")
endif()

# Set up paths
set(CHRONO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/chrono)
set(CHRONO_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/chrono)
set(OPTIX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../extern/optix)

# Verify chrono source exists
if(NOT EXISTS "${CHRONO_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Chrono source not found at ${CHRONO_SOURCE_DIR}. Did you run 'git submodule update --init --recursive'?")
endif()

# Verify OptiX exists (optional but recommended)
if(NOT EXISTS "${OPTIX_DIR}/include")
    message(WARNING "OptiX not found at ${OPTIX_DIR}. Chrono will build without OptiX support.")
    set(OPTIX_AVAILABLE FALSE)
else()
    set(OPTIX_AVAILABLE TRUE)
    message(STATUS "Found OptiX at ${OPTIX_DIR}")
endif()

# Get ROS2 environment for chrono_ros_interfaces
get_target_property(CHRONO_ROS_INTERFACES_INCLUDE_DIRS chrono_ros_interfaces::chrono_ros_interfaces INTERFACE_INCLUDE_DIRECTORIES)
if(NOT CHRONO_ROS_INTERFACES_INCLUDE_DIRS)
    message(WARNING "Could not get chrono_ros_interfaces include directories")
endif()

# Base CMake arguments for both builds
set(CHRONO_BASE_ARGS
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_PREFIX}
    -DCH_ENABLE_MODULE_VEHICLE=ON
    -DCH_ENABLE_MODULE_ROS=ON
    -DCH_ENABLE_MODULE_SENSOR=ON
    -DCH_ENABLE_MODULE_PARSERS=ON
    -DCH_ENABLE_MODULE_IRRLICHT=ON
    -DBUILD_DEMOS=OFF
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
    -Dament_cmake_DIR=${ament_cmake_DIR}
    -Dchrono_ros_interfaces_DIR=${chrono_ros_interfaces_DIR}
)

# Add OptiX if available
if(OPTIX_AVAILABLE)
    list(APPEND CHRONO_BASE_ARGS -DOptiX_INSTALL_DIR=${OPTIX_DIR})
endif()

# Build configuration selection
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(BUILD_DEBUG TRUE)
    set(BUILD_RELEASE FALSE)
    message(STATUS "Building Chrono in Debug mode only")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    set(BUILD_DEBUG FALSE) 
    set(BUILD_RELEASE TRUE)
    message(STATUS "Building Chrono in Release mode only")
else()
    # Build both for maximum compatibility
    set(BUILD_DEBUG TRUE)
    set(BUILD_RELEASE TRUE)
    message(STATUS "Building Chrono in both Debug and Release modes")
endif()

# Debug build
if(BUILD_DEBUG)
    ExternalProject_Add(chrono_debug
        SOURCE_DIR ${CHRONO_SOURCE_DIR}
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/chrono-debug
        CMAKE_ARGS
            ${CHRONO_BASE_ARGS}
            -DCMAKE_BUILD_TYPE=Debug
            -DCMAKE_DEBUG_POSTFIX=_d
        BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel 12
        INSTALL_COMMAND ${CMAKE_COMMAND} --install .
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
    )
endif()

# Release build
if(BUILD_RELEASE)
    ExternalProject_Add(chrono_release
        SOURCE_DIR ${CHRONO_SOURCE_DIR}
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/chrono-release
        CMAKE_ARGS
            ${CHRONO_BASE_ARGS}
            -DCMAKE_BUILD_TYPE=Release
        BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel 12
        INSTALL_COMMAND ${CMAKE_COMMAND} --install .
        LOG_CONFIGURE ON
        LOG_BUILD ON
        LOG_INSTALL ON
        DEPENDS $<$<BOOL:${BUILD_DEBUG}>:chrono_debug>
    )
endif()

# Create a custom target that depends on the builds
if(BUILD_DEBUG AND BUILD_RELEASE)
    add_custom_target(chrono_build ALL DEPENDS chrono_debug chrono_release)
elseif(BUILD_DEBUG)
    add_custom_target(chrono_build ALL DEPENDS chrono_debug)
else()
    add_custom_target(chrono_build ALL DEPENDS chrono_release)
endif()

# Install the entire chrono installation to our install space
install(DIRECTORY ${CHRONO_INSTALL_PREFIX}/
    DESTINATION chrono
    USE_SOURCE_PERMISSIONS
    PATTERN "*.la" EXCLUDE  # Exclude libtool archive files
)

# Create a config file that downstream packages can use
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ChronoConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ChronoConfig.cmake
    @ONLY
)

install(FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/ChronoConfig.cmake
    DESTINATION chrono/cmake
)

# Register package with ament
ament_package()
