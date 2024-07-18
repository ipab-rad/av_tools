#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml
# Default in-vehicle rosbags directory
ROSBAGS_DIR=/recorded_datasets/edinburgh

# Function to print usage
usage() {
    echo "
Usage: runtime.sh [-b|bash] [-l|--local] [--path | -p ] [--help | -h]

Options:
    -b | bash       Open bash in docker container
    -l | --local    Use default local cyclone_dds.xml config
                    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -p | --path   ROSBAGS_DIR_PATH
                    Specify path to store recorded rosbags
    -h | --help     Display this help message and exit.
    "
    exit 1
}


# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
            # Option to specify path
        -l|--local)
            if [[ -n "$2" && "$2" != -*  && "$2" != "bash" ]]; then
                CYCLONE_DIR="$2"
                shift
            fi
            CYCLONE_VOL="-v $CYCLONE_DIR:/opt/ros_ws/cyclone_dds.xml"
            ;;
        -p|--path)
            if [[ -n "$2" && "$2" != -* ]]; then
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

# Verify CYCLONE_DIR exists
if [ -n "$CYCLONE_VOL" ]; then
    if [ ! -f "$CYCLONE_DIR" ]; then
        echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
        exit 1
    fi
fi

# Verify ROSBAGS_DIR exists
if [ ! -d "$ROSBAGS_DIR" ]; then
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path to store rosbags"
    exit 1
fi

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t av_tools:latest \
    -f Dockerfile --target runtime .

# Run docker image
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    $CYCLONE_VOL \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    -v /etc/localtime:/etc/localtime:ro \
    av_tools:latest $BASH_CMD
