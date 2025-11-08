cmake_minimum_required(VERSION 3.20)
project(chrono_wrapper)

# Find ament cmake for ROS2
find_package(ament_cmake REQUIRED)

# Find chrono_ros_interfaces first
find_package(chrono_ros_interfaces REQUIRED)

# Include ExternalProject to build the chrono library and keep it separate from colcon build system
include(ExternalProject)

# Set up paths
set(CHRONO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/chrono)
set(CHRONO_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/chrono)
set(OPTIX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../extern/optix)

# Verify chrono source exists
if(NOT EXISTS "${CHRONO_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Chrono source not found at ${CHRONO_SOURCE_DIR}. Did you run 'git submodule update --init --recursive'?")
endif()

# Verify OptiX exists (optional)
if(NOT EXISTS "${OPTIX_DIR}/include")
    message(FATAL_ERROR "OptiX not found at ${OPTIX_DIR}. Consult ${OPTIX_DIR}/README.txt for more information")
endif()

# Just build release for now to simplify debugging
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
    -DOptiX_INSTALL_DIR=${OPTIX_DIR}
)

# Build
ExternalProject_Add(chrono_external
    SOURCE_DIR ${CHRONO_SOURCE_DIR}
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/chrono-build
    CMAKE_ARGS
        ${CHRONO_BASE_ARGS}
        -DCMAKE_BUILD_TYPE=${DCMAKE_BUILD_TYPE}
        -DCMAKE_DEBUG_POSTFIX=_d
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --parallel 12
    INSTALL_COMMAND ${CMAKE_COMMAND} --install .
    LOG_CONFIGURE ON
    LOG_BUILD ON
    LOG_INSTALL ON
)

# Create a custom target
add_custom_target(chrono_build ALL DEPENDS chrono_external)

# Install the chrono installation to our install space
install(DIRECTORY ${CHRONO_INSTALL_PREFIX}/
    DESTINATION chrono
    USE_SOURCE_PERMISSIONS
    PATTERN "*.la" EXCLUDE
)

# Register package with ament
ament_package()
