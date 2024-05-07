#!/bin/bash

# Log colors
CYAN="\033[0;36m"
MAGENTA="\033[0;35m"
NO_COLOR="\033[0m"

# Define the output directory
OUTPUT_DIR="$ROS_WS/rosbags"

# Generate a date and time prefix in the format YYYY_MM_DD-HH_MM_SS
DATE_PREFIX=$(date "+%Y_%m_%d-%H_%M_%S")

# Define default behavior if no file is provided
if [ "$#" -eq 0 ]; then
    echo -e "No topics list file provided. ${CYAN}Recording all topics.${NO_COLOR}"
    ros2 bag record -s mcap --all --max-cache-size 1048576000 \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -d 10740000000 \
        -o "$OUTPUT_DIR/${DATE_PREFIX}_sensor_recording"
    exit 0
elif [ "$#" -ne 1 ]; then
    echo "Usage: $0 [path-to-topics-list-file]"
    exit 1
fi

# Path to the topics list file
TOPICS_LIST_FILE="$1"

# Check if the topics list file exists
if [ ! -f "$TOPICS_LIST_FILE" ]; then
    echo "Error: File '$TOPICS_LIST_FILE' not found!"
    exit 1
fi

# Read topics into an array
readarray -t TOPICS < "$TOPICS_LIST_FILE"

# Start recording the topics
if [ ${#TOPICS[@]} -eq 0 ]; then
    echo "No topics found in the file. Stopping recording."
    exit 1
else
    echo -e "Recording topcis from ${MAGENTA}$TOPICS_LIST_FILE${NO_COLOR}"
    ros2 bag record -s mcap --max-cache-size 1048576000 \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -d 10740000000 \
        -o "$OUTPUT_DIR/${DATE_PREFIX}_sensor_recording" "${TOPICS[@]}"
fi
