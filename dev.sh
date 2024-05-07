#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools_humble:latest \
    -f Dockerfile --target dev .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

# Create a dir to store rosbags
mkdir -p rosbags

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    -v /recorded_datasets/edinburgh:/opt/ros_ws/rosbags \
    -v $SCRIPT_DIR/scripts/container_tools:/opt/ros_ws/container_tools \
    -v $SCRIPT_DIR/config:/opt/ros_ws/config \
    av_tools_humble:latest
