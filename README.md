# ros-px4-bridge-docker

Core PX4-in-Docker functionality for running PX4 SITL simulations with Gazebo and ROS 2.

## Overview

This repository provides the foundational scripts and configuration for running PX4 Autopilot in a containerized environment with:
- PX4 Autopilot SITL (Software In The Loop)
- Gazebo Garden simulator
- ROS 2 Humble
- Micro XRCE-DDS Agent for PX4-ROS communication

## Structure

```
docker-px4-core/
├── scripts/
│   ├── install_dependencies.sh    # Install system dependencies
│   ├── install_gazebo.sh          # Install Gazebo Garden
│   ├── install_dds_agent.sh       # Install Micro XRCE-DDS Agent
│   ├── prepare_workspace.sh       # Set up PX4 and ROS workspace
│   ├── build_px4.sh               # Build PX4 Autopilot
│   ├── build_ros2.sh              # Build ROS 2 workspace
│   ├── run_dds_agent.sh           # Start DDS agent
│   └── run_simulation.sh          # Start PX4 SITL simulation
├── README.md
└── LICENSE
```

## Usage

### Basic Setup

```bash
# Install all dependencies
./scripts/install_dependencies.sh
./scripts/install_gazebo.sh
./scripts/install_dds_agent.sh

# Prepare workspace (clones PX4 and ROS packages)
./scripts/prepare_workspace.sh

# Build everything
./scripts/build_px4.sh
./scripts/build_ros2.sh
```

### Running Simulation

```bash
# Terminal 1: Start DDS agent
./scripts/run_dds_agent.sh

# Terminal 2: Start PX4 simulation
./scripts/run_simulation.sh
```

## Environment Variables

- `PX4_SIM_SPEED_FACTOR`: Simulation speed multiplier (default: 2)
- `PX4_GZ_MODEL`: Drone model to use (default: x500)
- `PX4_UXRCE_DDS_NS`: DDS namespace (default: fmu)
- `PX4_UXRCE_DDS_PORT`: DDS port (default: 8888)
- `HEADLESS`: Run simulation without GUI (default: 1)

## Integration

This module can be used as a git submodule in other projects that need PX4 simulation capabilities.

```bash
git submodule add https://github.com/derickcoder44/docker-px4-core.git
```

## License

See LICENSE file for details.
