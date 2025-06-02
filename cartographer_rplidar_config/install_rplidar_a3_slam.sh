#!/bin/bash
# RPLIDAR A3 Cartographer SLAM - Complete Installation Script
# This script sets up everything needed for SLAM with RPLIDAR A3

set -e

echo "=== RPLIDAR A3 Cartographer SLAM Installation ==="
echo "This script will install and configure a complete SLAM system using:"
echo "• RPLIDAR A3 (optimized for 16m range, no IMU/odometry needed)"
echo "• Google Cartographer for 2D SLAM"
echo "• ROS2 Humble"
echo

# Check if running on Ubuntu 22.04
if ! grep -q "22.04" /etc/os-release; then
    echo "⚠ Warning: This script is designed for Ubuntu 22.04. Continue anyway? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if ROS2 Humble is installed
echo "1. Checking ROS2 Humble installation..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS2 Humble found"
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 Humble not found. Please install ROS2 Humble first:"
    echo "   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/ros2_ws"
echo "2. Setting up workspace at $WORKSPACE_DIR..."
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR/src"
    echo "✓ Created workspace directory"
else
    echo "✓ Workspace directory exists"
fi

cd "$WORKSPACE_DIR"

# Install required system packages
echo "3. Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libceres-dev \
    liblua5.3-dev

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo "4. Initializing rosdep..."
    sudo rosdep init
    rosdep update
else
    echo "4. Updating rosdep..."
    rosdep update
fi

# Clone required packages
echo "5. Cloning required packages..."
cd src

# Clone Cartographer if not exists
if [ ! -d "cartographer" ]; then
    echo "  - Cloning Cartographer..."
    git clone https://github.com/cartographer-project/cartographer.git
else
    echo "  ✓ Cartographer already exists"
fi

# Clone Cartographer ROS if not exists
if [ ! -d "cartographer_ros" ]; then
    echo "  - Cloning Cartographer ROS..."
    git clone https://github.com/cartographer-project/cartographer_ros.git
else
    echo "  ✓ Cartographer ROS already exists"
fi

# Clone SLLIDAR ROS2 if not exists
if [ ! -d "sllidar_ros2" ]; then
    echo "  - Cloning SLLIDAR ROS2..."
    git clone https://github.com/Slamtec/sllidar_ros2.git
else
    echo "  ✓ SLLIDAR ROS2 already exists"
fi

# Create the cartographer_rplidar_config package
PACKAGE_DIR="cartographer_rplidar_config"
echo "6. Creating RPLIDAR A3 configuration package..."
if [ ! -d "$PACKAGE_DIR" ]; then
    mkdir -p "$PACKAGE_DIR"/{config,launch}
    echo "✓ Created package directories"
else
    echo "✓ Package directories exist"
fi

# Create package.xml
cat > "$PACKAGE_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cartographer_rplidar_config</name>
  <version>1.0.0</version>
  <description>RPLIDAR A3 configuration for Google Cartographer SLAM</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>cartographer_ros</depend>
  <depend>sllidar_ros2</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz2</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > "$PACKAGE_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(cartographer_rplidar_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
EOF

# Create Cartographer configuration for RPLIDAR A3
cat > "$PACKAGE_DIR/config/rplidar_a3.lua" << 'EOF'
-- Copyright 2025 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- RPLIDAR A3 specific optimizations
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 16.  -- RPLIDAR A3 max range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Enhanced scan matching for no-odometry setup
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Pose graph optimization
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 90

-- Additional constraint builder settings for better loop closure
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03

-- Fast correlative scan matcher for loop closure
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1

return options
EOF

echo "7. Creating launch files..."

# Create launch file for SLAM only
cat > "$PACKAGE_DIR/launch/rplidar_a3_cartographer.launch.py" << 'EOF'
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('cartographer_rplidar_config').find('cartographer_rplidar_config')
    
    # Configuration file paths
    configuration_directory = os.path.join(pkg_share, 'config')
    configuration_basename = 'rplidar_a3.lua'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot description (simple TF tree for base_link -> laser)
    robot_description = '''<?xml version="1.0"?>
    <robot name="rplidar_robot">
      <link name="base_link"/>
      <link name="laser"/>
      <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser"/>
        <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
      </joint>
    </robot>'''
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # RPLIDAR node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'auto_standby': True,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
            ],
        ),
        
        # Cartographer occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])
EOF

# Create launch file with RViz
cat > "$PACKAGE_DIR/launch/rplidar_a3_cartographer_rviz.launch.py" << 'EOF'
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('cartographer_rplidar_config').find('cartographer_rplidar_config')
    rviz_config_file = os.path.join(pkg_share, 'config', 'cartographer_rplidar.rviz')
    
    return LaunchDescription([
        # Include the main SLAM launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('cartographer_rplidar_config'), 
                '/launch/rplidar_a3_cartographer.launch.py'
            ])
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
EOF

echo "8. Creating RViz configuration..."

# Create RViz configuration
cat > "$PACKAGE_DIR/config/cartographer_rplidar.rviz" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /LaserScan1
        - /Map1
        - /Map1/Topic1
      Splitter Ratio: 0.5
    Tree Height: 719
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        base_link:
          Value: true
        laser:
          Value: true
        map:
          Value: true
        odom:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          odom:
            base_link:
              laser:
                {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 4.71238899230957
Window Geometry:
  Displays:
    collapsed: false
  Height: 1016
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001560000039efc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000039e000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530063007200650065006e0100000000000000000000000000000000fb0000000800540069006d00650100000000000004500000000000000000000000000000039e0000000000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1920
  X: 1920
  Y: 27
EOF

echo "9. Creating utility scripts..."

# Create map saving script
cat > "$PACKAGE_DIR/save_map.sh" << 'EOF'
#!/bin/bash
# Script to save the current map

if [ $# -eq 0 ]; then
    echo "Usage: $0 <map_name>"
    echo "Example: $0 my_office_map"
    exit 1
fi

MAP_NAME="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Saving map as: $MAP_NAME"
echo "Location: $SCRIPT_DIR/maps/"

# Create maps directory if it doesn't exist
mkdir -p "$SCRIPT_DIR/maps"

# Save the map
cd "$SCRIPT_DIR/maps"
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"

if [ $? -eq 0 ]; then
    echo "✓ Map saved successfully!"
    echo "Files created:"
    echo "  - ${MAP_NAME}.pgm (map image)"
    echo "  - ${MAP_NAME}.yaml (map metadata)"
    ls -la "${MAP_NAME}."*
else
    echo "❌ Failed to save map. Make sure Cartographer is running."
    exit 1
fi
EOF

chmod +x "$PACKAGE_DIR/save_map.sh"

# Create test script
cat > "$PACKAGE_DIR/test_slam.sh" << 'EOF'
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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
cd "$WORKSPACE_DIR"
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
echo "  cd $(dirname "$SCRIPT_DIR")/cartographer_rplidar_config && ./save_map.sh my_map_name"
echo
echo "Key features of this setup:"
echo "• RPLIDAR A3 optimized (16m range, 256000 baudrate)"
echo "• No IMU or wheel odometry required"
echo "• Pure laser scan matching SLAM"
echo "• Pre-configured RViz visualization"
echo
echo "Happy mapping!"
EOF

chmod +x "$PACKAGE_DIR/test_slam.sh"

# Install dependencies and build
echo "10. Installing ROS dependencies..."
cd "$WORKSPACE_DIR"
rosdep install --from-paths src --ignore-src -r -y

echo "11. Building packages..."
colcon build --packages-select sllidar_ros2 cartographer_rplidar_config --symlink-install

echo "12. Setting up permissions..."
# Add user to dialout group for USB access
sudo usermod -a -G dialout $USER

echo "13. Creating setup script..."
cat > "setup_slam.sh" << EOF
#!/bin/bash
# Quick setup for RPLIDAR A3 SLAM

# Set USB permissions
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true

# Source workspace
cd "$WORKSPACE_DIR"
source install/setup.bash

echo "RPLIDAR A3 SLAM system ready!"
echo "Run: ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py"
EOF

chmod +x setup_slam.sh

echo
echo "=== Installation Complete! ==="
echo
echo "✓ All packages installed and configured"
echo "✓ RPLIDAR A3 optimized configuration created"
echo "✓ Launch files and utilities ready"
echo "✓ User added to dialout group (please log out and back in)"
echo
echo "To test the system:"
echo "  $WORKSPACE_DIR/src/cartographer_rplidar_config/test_slam.sh"
echo
echo "To start SLAM:"
echo "  cd $WORKSPACE_DIR && source install/setup.bash"
echo "  ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py"
echo
echo "Happy mapping!"
