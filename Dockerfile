FROM ros:humble-ros-base-jammy AS base

# Install key dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get -y --quiet --no-install-recommends install \
        ros-"$ROS_DISTRO"-can-msgs \
        ros-"$ROS_DISTRO"-dataspeed-ulc-msgs \
        ros-"$ROS_DISTRO"-dbw-ford-msgs \
        ros-"$ROS_DISTRO"-ffmpeg-image-transport \
        ros-"$ROS_DISTRO"-flir-camera-msgs \
        ros-"$ROS_DISTRO"-foxglove-bridge \
        ros-"$ROS_DISTRO"-gps-msgs \
        ros-"$ROS_DISTRO"-image-transport \
        ros-"$ROS_DISTRO"-image-transport-plugins \
        ros-"$ROS_DISTRO"-mcap-vendor \
        ros-"$ROS_DISTRO"-microstrain-inertial-msgs \
        ros-"$ROS_DISTRO"-nmea-msgs \
        ros-"$ROS_DISTRO"-novatel-gps-msgs \
        ros-"$ROS_DISTRO"-ouster-msgs \
        ros-"$ROS_DISTRO"-radar-msgs \
        ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
        ros-"$ROS_DISTRO"-rosbag2-storage-mcap \
        ros-"$ROS_DISTRO"-velodyne-msgs \
        ros-"$ROS_DISTRO"-geographic-msgs \
        python3-pip \
        python3-vcstool \
    && pip install mcap colorama \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./cyclone_dds.xml $ROS_WS/

# Configure Cyclone cfg file
ENV CYCLONEDDS_URI=file://${ROS_WS}/cyclone_dds.xml

# Enable ROS log colorised output
ENV RCUTILS_COLORIZED_OUTPUT=1

# Copy tools scripts and config
COPY scripts/container_tools $ROS_WS/container_tools
COPY config                  $ROS_WS/config

# Add tools to PATH
RUN echo "export PATH=$ROS_WS/container_tools:$PATH " >> /root/.bashrc &&\
    # Add sourcing local workspace command to bashrc for
    #    convenience when running interactively
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Create dep_ws
ENV DEP_WS=/opt/dep_ws
WORKDIR $DEP_WS

# Clone repos and build autoware msgs
COPY autoware_msgs.repos $DEP_WS/
RUN mkdir src && vcs import src < autoware_msgs.repos \
    && . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ./src ./build ./log \
    && echo "source $DEP_WS/install/setup.bash" >> /root/.bashrc

# Come back to ros_ws
WORKDIR $ROS_WS

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Nothing to build from source

# -----------------------------------------------------------------------

FROM prebuilt AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
        # ROS Rqt graph \
        ros-"$ROS_DISTRO"-rqt-graph \
        # Plot juggler
        ros-"$ROS_DISTRO"-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    source install/setup.bash"' >> /root/.bashrc

# Enter bash for clvelopment
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base AS runtime

# Start recording a rosbag by default
CMD ["/opt/ros_ws/container_tools/record_rosbag.sh"]
