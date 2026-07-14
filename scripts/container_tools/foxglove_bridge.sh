#!/bin/bash

# Source dependency msg pkgs before running foxglove bridge
. /opt/dep_ws/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    address:=172.31.0.1 send_buffer_limit:=1000000000 num_threads:=0
