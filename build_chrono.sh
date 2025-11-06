#!/usr/bin/env bash
set -e

# Get the directory of the script, resolving symlinks
SIM_TIRES_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Ensure we're in the expected directory
echo "Running build_chrono.sh from $SIM_TIRES_DIR..."
cd $SIM_TIRES_DIR

# Source the ROS environment
echo "Sourcing Jazzy ROS environment..."
source /opt/ros/jazzy/setup.sh

# Build chrono_ros_interfaces first before building the main chrono library
echo "Building chrono_ros_interfaces..."
colcon build --base-paths extern/chrono_ros_interfaces/

# Source the installation
echo "Sourcing installation..."
source install/setup.sh

# Generate debug configuration
echo "Generating Chrono Debug..."
cmake -S extern/chrono -B extern/chrono/build-dbg \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_INSTALL_PREFIX=$SIM_TIRES_DIR/install/chrono \
  -DCMAKE_C_COMPILER=gcc-12 \
  -DCMAKE_CXX_COMPILER=g++-12 \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_MODULE_ROS=ON \
  -DCH_ENABLE_MODULE_SENSOR=ON \
  -DCH_ENABLE_MODULE_PARSERS=ON \
  -DCH_ENABLE_MODULE_IRRLICHT=ON \
  -DOptiX_INSTALL_DIR=$SIM_TIRES_DIR/extern/optix \
  -DCMAKE_DEBUG_POSTFIX=_d \
  -DBUILD_DEMOS=OFF

# Build debug chrono
echo "Building Chrono Debug..."
cmake --build extern/chrono/build-dbg -j12

# Install debug chrono
echo "Installing Chrono Debug..."
cmake --install extern/chrono/build-dbg

# Generate release configuration
echo "Generating Chrono Release..."
cmake -S extern/chrono -B extern/chrono/build-rel \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$SIM_TIRES_DIR/install/chrono \
  -DCMAKE_C_COMPILER=gcc-12 \
  -DCMAKE_CXX_COMPILER=g++-12 \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_MODULE_ROS=ON \
  -DCH_ENABLE_MODULE_SENSOR=ON \
  -DCH_ENABLE_MODULE_PARSERS=ON \
  -DCH_ENABLE_MODULE_IRRLICHT=ON \
  -DOptiX_INSTALL_DIR=$SIM_TIRES_DIR/extern/optix \
  -DBUILD_DEMOS=OFF

# Build release chrono
echo "Building Chrono Release..."
cmake --build extern/chrono/build-rel -j12

# Install release chrono
echo "Installing Chrono Release..."
cmake --install extern/chrono/build-rel

echo "Finished Successfully!"
