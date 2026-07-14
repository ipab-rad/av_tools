FROM ros:jazzy-ros-base-noble AS base

# Install key dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get -y --quiet --no-install-recommends install \
        ros-"$ROS_DISTRO"-can-msgs \
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
        ros-"$ROS_DISTRO"-radar-msgs \
        ros-"$ROS_DISTRO"-rosbag2-storage-mcap \
        ros-"$ROS_DISTRO"-velodyne-msgs \
        ros-"$ROS_DISTRO"-geographic-msgs \
        ros-"$ROS_DISTRO"-pandar-msgs \
        ros-"$ROS_DISTRO"-autoware-*-msgs \
        python3-pip \
        python3-vcstool \
        python3-colorama \
        # Install Zenoh ROS2 RMW
        ros-"$ROS_DISTRO"-rmw-zenoh-cpp \
    && pip install --no-cache-dir --break-system-packages mcap \
    && rm -rf /var/lib/apt/lists/*

# Install local dependencies ("|| :" suppresses error if no *.deb found)
COPY ./deps /opt/ros_ws/deps
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        dpkg -i /opt/ros_ws/deps/*.deb \
    && rm -rf /var/lib/apt/lists/* || :

# Setup ROS workspace folder
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Get jazzy source rmw_zenoh and patch source via custom files
# This will become unnecessary when this PR is merged and backported to Jazzy
# https://github.com/ros2/rmw_zenoh/pull/1005
RUN mkdir "$ROS_WS"/src -p \
    && git clone https://github.com/assistive-autonomy/rmw_zenoh -b "$ROS_DISTRO" "$ROS_WS"/src/rmw_zenoh \
    && apt-get update && DEBIAN_FRONTEND=noninteractive \
    && rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y \
    && rm -rf /var/lib/apt/lists/*

# Setup Zenoh ROS2 RMW
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Enable ROS log colorised output
ENV RCUTILS_COLORIZED_OUTPUT=1

# Copy tools scripts and config
COPY scripts/container_tools $ROS_WS/container_tools
COPY config                  $ROS_WS/config

# Set tools autocomplete
RUN echo "source $ROS_WS/container_tools/_tools_autocomplete.sh" >> /etc/bash.bashrc

# Add tools to PATH
RUN echo "export PATH=$ROS_WS/container_tools:$PATH " >> /etc/bash.bashrc &&\
    # Add sourcing local workspace command to bashrc for
    #    convenience when running interactively
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

# Create dep_ws
ENV DEP_WS=/opt/dep_ws
WORKDIR $DEP_WS

# Clone Humble Dataspeed repos, to be compiled in Jazzy
RUN git clone https://bitbucket.org/DataspeedInc/dbw_ros.git /opt/dbw_ros

# Move to src only necessary msg pkgs
RUN mkdir -p $DEP_WS/src \
 && mv /opt/dbw_ros/dbw1/dataspeed_ulc_msgs $DEP_WS/src/ \
 && mv /opt/dbw_ros/dbw1/dbw_ford_msgs $DEP_WS/src/

# Compile msg pkgs from source
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ./src ./build ./log \
    && echo "source $DEP_WS/install/setup.bash" >> /etc/bash.bashrc

# Come back to ros_ws
WORKDIR $ROS_WS

# Create username
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=lxo

# Remove annoying default ubuntu user with conflicting id
RUN touch /var/mail/ubuntu && chown ubuntu /var/mail/ubuntu && userdel -r ubuntu

RUN groupadd -g $GROUP_ID $USERNAME && \
    useradd -u $USER_ID -g $GROUP_ID -m -l $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

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
    source install/setup.bash"' >> /etc/bash.bashrc

# Enter bash for development
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base AS runtime

# Start recording a rosbag by default
CMD ["/opt/ros_ws/container_tools/record_rosbag.sh"]
