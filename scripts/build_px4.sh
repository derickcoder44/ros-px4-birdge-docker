#!/bin/bash
# Build PX4 Autopilot
set -e

WORKSPACE=${WORKSPACE:-$HOME/workspace}
PX4_DIR=$WORKSPACE/PX4-Autopilot

echo "Building PX4 Autopilot..."

# Set up environment variables
export PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-2}
export PX4_GZ_MODEL=${PX4_GZ_MODEL:-x500}
export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/build_gz-garden

cd $PX4_DIR

# Build PX4 SITL for Gazebo
echo "Building PX4 SITL target..."
HEADLESS=1 make px4_sitl_default

# Build Gazebo models
echo "Building Gazebo Garden models..."
make px4_sitl_default gz_x500

echo "PX4 Autopilot build completed!"
