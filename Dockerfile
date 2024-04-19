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
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS

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

# Add sourcing local workspace command to bashrc for 
#    convenience when running interactively
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash"' >> /root/.bashrc

# Enter bash for clvelopment
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base as runtime

# Launch foxglove by default
CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", \
    "send_buffer_limit:=1000000000", "num_threads:=4"]
