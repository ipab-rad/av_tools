#!/bin/bash

# Configuration variables
VEHICLE_USER="seb_adm"
VEHICLE_HOST="core5.vp.five.ai"
REMOTE_DIR="/data_ssd/rad/tartan_docker_images/"

# Directory to save tar files locally
LOCAL_SAVE_DIR="$HOME/tartan_docker_images"

# ANSI colors
GREEN="\033[0;32m"
CYAN="\033[0;36m"
MAGENTA="\033[0;35m"
BMAGENTA="\033[1;35m"
YELLOW="\033[1;33m"
WHITE="\033[1;37m"
RED="\033[0;31m"
BLUE="\033[0;34m"
NO_COLOR="\033[0m"

# Check if the image name was provided as an argument
if [ $# -ne 1 ]; then
    echo -e "${YELLOW}Usage: $0 <docker-image>${NO_COLOR}"
    exit 1
fi

IMAGE_NAME=$1
TAR_FILE="${IMAGE_NAME//[:\/]/_}.tar"  # Create a filename from image name by replacing potential problematic characters

# Ensure the local directory for saving Docker tar files exists
if [ ! -d "$LOCAL_SAVE_DIR" ]; then
    echo -e "${CYAN}Creating directory $LOCAL_SAVE_DIR...${NO_COLOR}"
    mkdir -p "$LOCAL_SAVE_DIR"
fi

# Full path for the tar file
FULL_TAR_PATH="$LOCAL_SAVE_DIR/$TAR_FILE"

# Check if the tar file already exists
if [ -f "$FULL_TAR_PATH" ]; then
    echo -e "${WHITE}File ${CYAN}$FULL_TAR_PATH ${WHITE}already exists. Do you want to overwrite?${NO_COLOR}"
    read -p "(y/N): " response
    case "$response" in
        [yY][eE][sS]|[yY])
            echo -e "${YELLOW}Overwriting the file...${NO_COLOR}"
            echo -e "${WHITE}Saving Docker image ${MAGENTA}$IMAGE_NAME ${WHITE}to a tar file at ${CYAN}$FULL_TAR_PATH...${NO_COLOR}"
            docker save $IMAGE_NAME > "$FULL_TAR_PATH"
            ;;
        *)
            echo -e "${YELLOW}Not overwriting the file. Proceeding with upload...${NO_COLOR}"
            ;;
    esac
else
    echo -e "${CYAN}Saving Docker image $IMAGE_NAME to a tar file at $FULL_TAR_PATH...${NO_COLOR}"
    docker save $IMAGE_NAME > "$FULL_TAR_PATH"
fi

# Use rsync to transfer the tar file to the remote directory
echo -e "${WHITE}Transferring ${CYAN}$FULL_TAR_PATH ${WHITE}to ${BLUE}$VEHICLE_USER@$VEHICLE_HOST:$REMOTE_DIR${NO_COLOR}"
rsync "$FULL_TAR_PATH" "${VEHICLE_USER}@${VEHICLE_HOST}:${REMOTE_DIR}"

# Check if rsync succeeded
if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to transfer file. Unable to reach host or other transfer error.${NO_COLOR}"
    # Handle error as needed
    exit 1
else
    echo -e "${WHITE}Image ${MAGENTA}$IMAGE_NAME ${WHITE}uploaded successfully to ${BLUE}$VEHICLE_HOST.${NO_COLOR}"
    echo -e "Run \`docker load -i ${REMOTE_DIR}\` on remote"

fi
