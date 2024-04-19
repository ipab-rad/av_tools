FROM ros:humble-ros-base-jammy AS base

# Install key dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        ros-"$ROS_DISTRO"-rosbag2-storage-mcap \
        ros-"$ROS_DISTRO"-mcap-vendor \
        ros-"$ROS_DISTRO"-foxglove-bridge \
        ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
        ros-"$ROS_DISTRO"-can-msgs \
        ros-"$ROS_DISTRO"-dataspeed-ulc-msgs \
        ros-"$ROS_DISTRO"-dbw-ford-msgs \
        ros-"$ROS_DISTRO"-gps-msgs \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS

# Copy tools scripts and config
COPY scripts/container_tools $ROS_WS/container_tools
COPY config                  $ROS_WS/config

# Add tools to PATH
RUN echo "export PATH=$ROS_WS/container_tools:$PATH " >> /root/.bashrc

# Add sourcing local workspace command to bashrc for 
#    convenience when running interactively
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Nothing to build from source

# -----------------------------------------------------------------------

FROM base AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash"' >> /root/.bashrc

# Enter bash for clvelopment
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base as runtime

# Start recording a rosbag by default
CMD $ROS_WS/container_tools/record_rosbag
