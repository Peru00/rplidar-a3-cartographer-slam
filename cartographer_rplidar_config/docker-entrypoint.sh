#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash

# If RPLIDAR device is specified, ensure it has correct permissions
if [ -e "/dev/ttyUSB0" ]; then
    echo "RPLIDAR detected at /dev/ttyUSB0"
fi

exec "$@"
