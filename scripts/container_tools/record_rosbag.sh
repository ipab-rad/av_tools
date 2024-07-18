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
    echo "Usage: record_rosbag.sh [options] [path-to-topics-list-file.txt]"
    echo ""
    echo "Options:"
    echo "  -n, --name    Set the rosbag name (default is 'sensor_recording')."
    echo "  -h, --help    Display this help message and exit."
    echo ""
    echo "Examples:"
    echo "  record_rosbag.sh  -n custom_name topics_list.txt"
    echo "  record_rosbag.sh  --name custom_name topics_list.txt"
    echo "  record_rosbag.sh  topics_list.yaml"
    echo "  record_rosbag.sh  (record all topics)"
    exit 0
}

# Initialize variables
TOPICS_LIST_FILE=""
ROSBAG_SUFFIX="sensor_recording"

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--name)
            ROSBAG_SUFFIX="$2"
            shift
            ;;
        -h|--help)
            help
            ;;
        *)
            TOPICS_LIST_FILE="$1"
            ;;
    esac
    shift
done

# Define default behavior if no file is provided
if [ -z "$TOPICS_LIST_FILE" ]; then
    echo -e "No topics list file provided. ${CYAN}Recording all topics.${NO_COLOR}"
    ros2 bag record -s mcap --all --max-cache-size 5000000000 \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -b 10740000000 \
        -o "$OUTPUT_DIR/${DATE_PREFIX}_all_topics"
    exit 0
fi

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
    echo -e "Recording topics from ${MAGENTA}$TOPICS_LIST_FILE${NO_COLOR}"
    ros2 bag record -s mcap --max-cache-size 5000000000 \
        --storage-config-file "$ROS_WS/config/mcap_cfg.yaml" \
        -b 10740000000 \
        -o "$OUTPUT_DIR/${DATE_PREFIX}_${ROSBAG_SUFFIX}" "${TOPICS[@]}"
fi
