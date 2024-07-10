#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh

# Function to print usage
usage() {
    echo "Usage: runtime.sh [--path | -p ] [--help | -h]"
    echo ""
    echo "Options:"
    echo "  --path, -p ROSBAGS_DIR_PATH"
    echo "                 Specify path to store recorded rosbags"
    echo "  --help, -h     Display this help message and exit."
    echo ""
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

# Initialise CMD as empty
CMD=""

# If an arg is defined, start container with bash
if [ -n "$1" ]; then
    CMD="bash"
fi

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools:latest \
    -f Dockerfile --target runtime .

# Run docker image
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v /etc/localtime:/etc/localtime:ro \
    av_tools:latest $CMD
