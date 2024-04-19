#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development 
# ----------------------------------------------------------------

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools_humble:latest-dev \
    -f Dockerfile --target dev .

# Create a dir to store rosbags
mkdir -p rosbags

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    -v ./rosbags:/opt/ros_ws/rosbags \
    -v ./scripts/container_tools:/opt/ros_ws/container_tools \
    -v ./config:/opt/ros_ws/config \
    av_tools_humble:latest-dev