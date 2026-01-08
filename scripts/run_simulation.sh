#!/bin/bash
# Start PX4 SITL simulation with Gazebo
set -e

WORKSPACE=${WORKSPACE:-$HOME/workspace}
PX4_DIR=$WORKSPACE/PX4-Autopilot
LOG=${LOG:-$HOME/logs/px4_gz.log}
mkdir -p "$(dirname "$LOG")"

echo "Starting PX4 SITL simulation..."

# Wait for DDS agent to be ready
for i in $(seq 1 120); do
  if pgrep -f MicroXRCEAgent >/dev/null || [ -f "$HOME/.microxrce_agent.pid" ]; then
    echo "MicroXRCEAgent is running, proceeding..."
    break
  fi
  echo "Waiting for MicroXRCEAgent to be ready..."
  sleep 1
done

# Set up environment variables
export PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-2}
export PX4_GZ_MODEL=${PX4_GZ_MODEL:-x500}
export PX4_UXRCE_DDS_NS=${PX4_UXRCE_DDS_NS:-fmu}
export PX4_UXRCE_DDS_PORT=${PX4_UXRCE_DDS_PORT:-8888}
export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/build_gz-garden
# Set custom world with camera following
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-x500_follow}
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(dirname "$0")/../worlds
export HEADLESS=${HEADLESS:-1}

cd $PX4_DIR

# Start PX4 simulation (uses pre-built binary, fast startup)
echo "Starting PX4 SITL with Gazebo Garden..."
HEADLESS=$HEADLESS make px4_sitl gz_x500 > "$LOG" 2>&1 &
PX4_PID=$!
echo $PX4_PID > /tmp/px4_pid.txt

echo "PX4 simulation started (PID: $PX4_PID)"
echo "Log file: $LOG"

# Wait for PX4 to start
for i in $(seq 1 60); do
  if [ -e /tmp/px4_pid.txt ] && kill -0 "$PX4_PID" >/dev/null 2>&1; then
    echo "PX4 SITL is running"
    break
  fi
  sleep 1
done
