#!/bin/bash

# Log colors
CYAN="\033[0;36m"
MAGENTA="\033[0;35m"
NO_COLOR="\033[0m"

# Define the output directory
OUTPUT_DIR="$ROS_WS/rosbags"

# Generate a date and time prefix in the format YYYY_MM_DD-HH_MM_SS
DATE_PREFIX=$(date "+%Y_%m_%d-%H_%M_%S")

# Function to display help
help() {
    echo "Usage: record_rosbag.sh [options]

    Options:
    --all       Record all available topics
    -f, --file TOPIC_LIST.txt
                Specify topic list file (Default: all_sensor_topics.txt)
    -n, --name ROSBAG_NAME
                Set the rosbag name (Default: all_sensors_recording).
    -h, --help  Display this help message and exit.
    "
    exit 0
}

TOPICS_LIST_FILE="$ROS_WS/config/all_sensor_topics.txt"
ROSBAG_SUFFIX="all_sensors_recording"
RECORD_ALL=""
MAX_CACHE_SIZE="5000000000"
MAX_BAG_SIZE="10740000000"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --all)
            RECORD_ALL="YES"
            ;;
        -f|--file)
            TOPICS_LIST_FILE="$2"
            shift
            ;;
        -n|--name)
            ROSBAG_SUFFIX="$2"
            shift
            ;;
        -h|--help)
            help
            ;;
        *)
            echo "Unsupported flag"
            help
            ;;
    esac
    shift
done

if [ ! -z "$RECORD_ALL" ]; then
    echo -e "${CYAN}Recording all topics.${NO_COLOR}"

    ROSBAG_PATH="$OUTPUT_DIR/${DATE_PREFIX}_all_topics"

    ros2 bag record -s mcap --all --max-cache-size $MAX_CACHE_SIZE \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -b $MAX_BAG_SIZE \
        -o "$ROSBAG_PATH"

    echo -e "Recording saved in: ${CYAN}${ROSBAG_PATH}${NO_COLOR}"
    exit 0
fi

# Check if the topics list file exists
if [ ! -f "$TOPICS_LIST_FILE" ]; then
    echo "Error: File '$TOPICS_LIST_FILE' not found!"
    exit 1
fi

# Read topics into an array
readarray -t TOPICS < "$TOPICS_LIST_FILE"

ROSBAG_PATH="${OUTPUT_DIR}/${DATE_PREFIX}_${ROSBAG_SUFFIX}"

if [ ${#TOPICS[@]} -eq 0 ]; then
    echo "No topics found in the file. Stopping recording."
    exit 1
else
    echo -e "Recording topics from ${MAGENTA}$TOPICS_LIST_FILE${NO_COLOR}"
    ros2 bag record -s mcap --max-cache-size $MAX_CACHE_SIZE \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -b $MAX_BAG_SIZE \
        -o $ROSBAG_PATH "${TOPICS[@]}"
fi

echo -e "Recording saved in: ${CYAN}${ROSBAG_PATH}${NO_COLOR}"
