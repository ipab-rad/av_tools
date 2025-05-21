#!/usr/bin/env python3

import subprocess
import time
from datetime import datetime

# Define the duration in hours
DURATION_HOURS = 16.0

def run_rosbag_record():
    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    rosbag_name = f"{timestamp}_lidars_static"
    rosbag_path = "/opt/ros_ws/rosbags/" + rosbag_name

    command = ["ros2", "bag", "record", "--no-discovery","-s", "mcap", "-o", rosbag_path, "/sensor/imu/front/data", "/sensor/imu/rear/data"]

    try:
        process = subprocess.Popen(command)
        time.sleep(0.23) # the recording doesn't start immediately.

        print(f"Recording until {DURATION_HOURS} hours passed")

        # Wait for the specified duration
        time.sleep(DURATION_HOURS * 3600)

        # Kill the process after the duration
        process.terminate()
        process.wait()
        print(f'Rosbag saved in: {rosbag_path}')


    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Stopping rosbag recording...")
        process.terminate()
        process.wait()

    except Exception as e:
        print(f"An error occurred: {e}")
        if process.poll() is None:  # If process is still running
            process.terminate()
            process.wait()

if __name__ == "__main__":
    run_rosbag_record()
