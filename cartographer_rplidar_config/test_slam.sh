#!/bin/bash
# Test script for RPLIDAR A3 Cartographer SLAM

echo "=== RPLIDAR A3 Cartographer SLAM Test ==="
echo

# Check if RPLIDAR is connected
echo "1. Checking RPLIDAR A3 connection..."
if ls /dev/ttyUSB* > /dev/null 2>&1; then
    echo "✓ RPLIDAR found:"
    ls -la /dev/ttyUSB*
else
    echo "⚠ No RPLIDAR detected. Please connect your RPLIDAR A3 and try again."
    exit 1
fi

echo

# Source workspace
echo "2. Sourcing ROS2 workspace..."
cd /home/peru/ros2_ws
source install/setup.bash
echo "✓ Workspace sourced"

echo

# Check if packages are available
echo "3. Verifying packages..."
if ros2 pkg list | grep -q cartographer_rplidar_config; then
    echo "✓ cartographer_rplidar_config package found"
else
    echo "❌ cartographer_rplidar_config package not found"
    exit 1
fi

if ros2 pkg list | grep -q sllidar_ros2; then
    echo "✓ sllidar_ros2 package found"
else
    echo "❌ sllidar_ros2 package not found"
    exit 1
fi

echo

echo "=== Setup Complete! ==="
echo
echo "To start SLAM mapping:"
echo "  ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py"
echo
echo "To save a map after mapping:"
echo "  cd /home/peru/ros2_ws/src/cartographer_rplidar_config && ./save_map.sh my_map_name"
echo
echo "Key features of this setup:"
echo "• RPLIDAR A3 optimized (16m range, 256000 baudrate)"
echo "• No IMU or wheel odometry required"
echo "• Pure laser scan matching SLAM"
echo "• Pre-configured RViz visualization"
echo
echo "Happy mapping!"
