#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

# Initialise CMD as empty
CMD=""

# If an arg is defined, start container with bash
if [ -n "$1" ]; then
    CMD="bash"
fi

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools_humble:latest \
    -f Dockerfile --target runtime .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

# Create a dir to store rosbags
mkdir -p $SCRIPT_DIR/rosbags

# Run docker image
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm  \
    -v $SCRIPT_DIR/rosbags:/opt/ros_ws/rosbags \
    av_tools_humble:latest $CMD
