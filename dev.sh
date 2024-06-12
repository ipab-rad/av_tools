#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh
# Default value for headless
headless=false

# Function to print usage
usage() {
    echo "Usage: dev.sh [--path | -p ] [--headless] [--help | -h]"
    echo ""
    echo "Options:"
    echo "  --path, -p ROSBAGS_DIR_PATH"
    echo "                 Specify path to store recorded rosbags"
    echo "  --headless     Run the Docker image without X11 forwarding"
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
        --headless) headless=true ;;
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


MOUNT_X=""
if [ "$headless" = "false" ]; then
    MOUNT_X="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix"
    xhost + >/dev/null
fi

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools:latest-dev \
    -f Dockerfile --target dev .

# Get the absolute path of the script
SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    ${MOUNT_X} \
    -e XAUTHORITY="${XAUTHORITY}" \
    -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v $SCRIPT_DIR/cyclone_dds.xml:/opt/ros_ws/cyclone_dds.xml \
    -v $SCRIPT_DIR/scripts/container_tools:/opt/ros_ws/container_tools \
    -v $SCRIPT_DIR/config:/opt/ros_ws/config \
    -v /etc/localtime:/etc/localtime:ro \
    av_tools:latest-dev
