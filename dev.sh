#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh

# Function to print usage
usage() {
    echo "Usage: dev.sh [-p|--path <absolute path to store rosbags>]"
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
            # Option to specify path
        -p|--path)
            if [ -n "$2" ]; then
                ROSBAGS_DIR="$2"
                shift
            else
                echo "Error: Argument for $1 is missing."
                usage
            fi
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Verify ROSBAGS_DIR exists
if [ ! -d "$ROSBAGS_DIR" ]; then
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path to store rosbags"
    exit 1
fi


# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools:latest-dev \
    -f Dockerfile --target dev .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v $SCRIPT_DIR/cyclone_dds.xml:/opt/ros_ws/cyclone_dds.xml \
    -v $SCRIPT_DIR/scripts/container_tools:/opt/ros_ws/container_tools \
    -v $SCRIPT_DIR/config:/opt/ros_ws/config \
    -v /etc/localtime:/etc/localtime:ro \
    av_tools:latest-dev
