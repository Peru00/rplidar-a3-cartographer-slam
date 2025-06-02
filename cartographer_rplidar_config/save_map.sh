#!/bin/bash

# Map Saving Script for Cartographer SLAM
# This script helps you save the map after SLAM mapping

echo "=========================================="
echo "Cartographer Map Saver"
echo "=========================================="

# Check if cartographer is running
if ! pgrep -f "cartographer_node" > /dev/null; then
    echo "ERROR: Cartographer node is not running!"
    echo "Please start SLAM mapping first with:"
    echo "ros2 launch cartographer_rplidar_config rplidar_a3_cartographer.launch.py"
    exit 1
fi

# Default map name
MAP_NAME="rplidar_a3_map"

# Check if user provided a map name
if [ $# -eq 1 ]; then
    MAP_NAME="$1"
fi

echo "Saving map as: $MAP_NAME"
echo "Location: $HOME/$MAP_NAME"

# Source the workspace
cd /home/peru/ros2_ws
source install/setup.bash

# Install nav2_map_server if not already installed
if ! ros2 pkg list | grep -q "nav2_map_server"; then
    echo "Installing nav2_map_server..."
    sudo apt install -y ros-humble-nav2-map-server
fi

# Save the map
echo "Saving map... Please wait..."
ros2 run nav2_map_server map_saver_cli -f "$HOME/$MAP_NAME"

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "Map saved successfully!"
    echo "=========================================="
    echo "Files created:"
    echo "  - $HOME/$MAP_NAME.pgm (map image)"
    echo "  - $HOME/$MAP_NAME.yaml (map metadata)"
    echo ""
    echo "You can use this map for navigation later!"
else
    echo "ERROR: Failed to save map. Please check if cartographer is running."
fi
