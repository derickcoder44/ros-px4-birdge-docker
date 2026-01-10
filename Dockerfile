FROM ghcr.io/sloretz/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV PX4_VERSION=release/1.14

# Install minimal dependencies for ROS2 bridge
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    build-essential \
    cmake \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# Configure ccache
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/root/.cache/ccache

# Install Micro-XRCE-DDS Agent
RUN cd /tmp && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Set up ROS2 workspace
WORKDIR /root/workspace
RUN mkdir -p ros2_ws/src

# Clone repos in separate layer for better caching
RUN cd ros2_ws/src && \
    git clone -b ${PX4_VERSION} https://github.com/PX4/px4_msgs.git && \
    git clone -b release/v1.14 https://github.com/PX4/px4_ros_com.git

# Build ROS 2 workspace with BuildKit cache mount for faster rebuilds
# Note: --symlink-install allows install/ to reference src/, so we keep both
RUN --mount=type=cache,target=/root/.cache/ccache,sharing=locked \
    . /opt/ros/humble/setup.sh && \
    cd ros2_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache || \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set up environment
ENV PX4_UXRCE_DDS_PORT=8888

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /root/workspace/ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
