#!/bin/bash

# RPLIDAR A3 Cartographer Setup Script
# This script helps you run SLAM with RPLIDAR A3 without IMU or wheel odometry

echo "=========================================="
echo "RPLIDAR A3 Cartographer SLAM Setup"
echo "=========================================="

# Check if RPLIDAR is connected
if [ ! -e /dev/ttyUSB0 ]; then
    echo "ERROR: RPLIDAR not found at /dev/ttyUSB0"
    echo "Please check if your RPLIDAR A3 is connected via USB"
    exit 1
fi

echo "RPLIDAR A3 detected at /dev/ttyUSB0"

# Set permissions for the device
echo "Setting permissions for RPLIDAR device..."
sudo chmod 666 /dev/ttyUSB0

# Source the workspace
echo "Sourcing ROS2 workspace..."
cd /home/peru/ros2_ws
source install/setup.bash

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Available launch commands:"
echo ""
echo "1. Start SLAM mapping (without RViz):"
echo "   ros2 launch cartographer_rplidar_config rplidar_a3_cartographer.launch.py"
echo ""
echo "2. Start SLAM mapping with RViz visualization:"
echo "   ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py"
echo ""
echo "3. Save the map when you're done mapping:"
echo "   ros2 run nav2_map_server map_saver_cli -f ~/my_map"
echo ""
echo "Note: Make sure to move your robot around slowly to build a good map!"
echo "The RPLIDAR A3 has a range of up to 16 meters."
echo ""
