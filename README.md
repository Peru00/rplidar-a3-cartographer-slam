# RPLIDAR A3 Cartographer SLAM - Complete Setup Package

A ready-to-use 2D SLAM system using **RPLIDAR A3** with **Google Cartographer** on **ROS2 Humble**. This package provides a complete configuration optimized for RPLIDAR A3 without requiring IMU or wheel odometry.

## 📋 Table of Contents
- [🚀 Quick Start (Automated)](#-quick-start-automated)
- [📚 Step-by-Step Installation Guide](#-step-by-step-installation-guide)
- [🐳 Docker Installation](#-docker-installation)
- [🔧 Manual Package Installation](#-manual-package-installation)
- [▶️ Usage](#️-usage)
- [🔧 System Requirements](#-system-requirements)

## 🚀 Quick Start (Automated)

### One-Command Installation (Recommended)

**For users who want everything installed automatically:**

```bash
curl -fsSL https://raw.githubusercontent.com/peru00/rplidar-cartographer-slam/main/install_rplidar_a3_slam.sh | bash
```

This script will:
- Install ROS2 Humble (if not present)
- Install all required dependencies
- Set up the RPLIDAR A3 SLAM package
- Verify the installation

## 📚 Step-by-Step Installation Guide

### Prerequisites
- Ubuntu 22.04 LTS (recommended)
- Internet connection
- USB port for RPLIDAR A3

### Step 1: Install ROS2 Humble

1. **Set up the ROS2 repository:**
```bash
# Ensure you have required packages
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. **Install ROS2 Humble:**
```bash
# Update package list
sudo apt update

# Install ROS2 Humble Desktop (includes RViz)
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
```

3. **Set up ROS2 environment:**
```bash
# Add to your .bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### Step 2: Install RPLIDAR/SLLIDAR Package

1. **Install the SLLIDAR ROS2 package:**
```bash
sudo apt update
sudo apt install -y ros-humble-sllidar-ros2
```

2. **Set up USB permissions for RPLIDAR A3:**
```bash
# Add yourself to dialout group
sudo usermod -a -G dialout $USER

# Create udev rule for RPLIDAR
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Note: You may need to logout and login again for group changes to take effect
```

### Step 3: Install Google Cartographer

1. **Install Cartographer packages:**
```bash
sudo apt update
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros
```

2. **Install additional dependencies:**
```bash
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher
```

### Step 4: Set Up ROS2 Workspace

1. **Create a ROS2 workspace:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. **Add workspace to .bashrc:**
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Install RPLIDAR A3 SLAM Package

1. **Clone the package:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/peru00/rplidar-cartographer-slam.git
mv rplidar-cartographer-slam cartographer_rplidar_config
```

2. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select cartographer_rplidar_config
source install/setup.bash
```

3. **Verify installation:**
```bash
# Run the test script
cd ~/ros2_ws/src/cartographer_rplidar_config
./test_slam.sh
```

### Step 6: Connect and Test RPLIDAR A3

1. **Connect your RPLIDAR A3 to USB port**

2. **Check device detection:**
```bash
# Check if device is detected
ls -la /dev/ttyUSB*
# Should show something like /dev/ttyUSB0

# Test RPLIDAR communication
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
```

3. **Launch SLAM system:**
```bash
# Launch SLAM with RViz visualization
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py

# Or launch SLAM only (without RViz)
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer.launch.py
```

## 🐳 Docker Installation

**For users who prefer containerized installation:**

1. **Install Docker:**
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Logout and login again
```

2. **Clone and build:**
```bash
git clone https://github.com/peru00/rplidar-cartographer-slam.git
cd rplidar-cartographer-slam
docker build -t rplidar-slam .
```

3. **Run with GUI support:**
```bash
# Allow X11 forwarding
xhost +local:docker

# Run container
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  rplidar-slam
```

## 🔧 Manual Package Installation

**For users with existing ROS2 Humble setup:**

1. **Copy package to your workspace:**
```bash
cp -r cartographer_rplidar_config /path/to/your/ros2_ws/src/
cd /path/to/your/ros2_ws
colcon build --packages-select cartographer_rplidar_config
source install/setup.bash
```

## ▶️ Usage

### Basic Operation

1. **Test Your Setup:**
```bash
cd ~/ros2_ws/src/cartographer_rplidar_config
./test_slam.sh
```

2. **Start SLAM with Visualization:**
```bash
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py
```

3. **Start SLAM Only (no visualization):**
```bash
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer.launch.py
```

4. **Move Your Robot/Sensor:**
   - Move slowly and smoothly for best results
   - Include loop closures (return to starting areas)
   - Avoid highly reflective surfaces

5. **Save Your Map:**
```bash
./save_map.sh my_office_map
```

### Custom Device Path
If your RPLIDAR A3 is not on `/dev/ttyUSB0`:
```bash
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py serial_port:=/dev/ttyUSB1
```

### View Topics and Data
```bash
# List available topics
ros2 topic list

# View laser scan data
ros2 topic echo /scan

# View map data
ros2 topic echo /map

# View transform tree
ros2 run tf2_tools view_frames
```

## 🔧 System Requirements

- **OS:** Ubuntu 22.04 LTS
- **ROS:** ROS2 Humble
- **Hardware:** RPLIDAR A3 connected via USB
- **RAM:** 4GB+ recommended
- **CPU:** Multi-core processor recommended

## 🎯 Key Features

- **✅ RPLIDAR A3 Optimized:** 16m range, 256000 baudrate, 10Hz scan frequency
- **✅ No IMU Required:** Pure laser-based SLAM using scan matching
- **✅ No Wheel Odometry:** Works without encoders or motion sensors
- **✅ Real-time Mapping:** Live visualization in RViz
- **✅ Loop Closure:** Automatic map correction when revisiting areas
- **✅ Map Saving:** Easy export to standard formats (.pgm, .yaml)

## 📦 Package Contents

```
cartographer_rplidar_config/
├── config/
│   ├── rplidar_a3.lua                      # Cartographer configuration
│   └── cartographer_rplidar.rviz           # RViz visualization setup
├── launch/
│   ├── rplidar_a3_cartographer.launch.py           # SLAM only
│   └── rplidar_a3_cartographer_rviz.launch.py      # SLAM + RViz
├── install_rplidar_a3_slam.sh              # Complete installation script
├── test_slam.sh                            # System verification
├── save_map.sh                             # Map saving utility
├── Dockerfile                              # Docker setup
└── README.md                               # This file
```

## 🚀 Usage

### 1. Test Your Setup
```bash
./test_slam.sh
```

### 2. Start SLAM with Visualization
```bash
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py
```

### 3. Move Your Robot
- Move slowly and smoothly for best results
- Include loop closures (return to starting areas)
- Avoid highly reflective surfaces

### 4. Save Your Map
```bash
./save_map.sh my_office_map
```

## 🔧 Configuration Highlights

### RPLIDAR A3 Specific Settings
- **Range:** 0.15m to 16.0m (full A3 capability)
- **Baudrate:** 256000 (optimal for A3)
- **Frame rate:** 10Hz standard mode
- **Auto-standby:** Enabled for motor protection

### Cartographer Optimizations
- **Enhanced scan matching** for no-odometry environments
- **Larger search windows** to compensate for drift
- **Optimized pose graph** parameters for better loop closure
- **Real-time correlative scan matcher** tuned for RPLIDAR A3

## 🛠 Troubleshooting

### Installation Issues

**ROS2 Installation Failed:**
```bash
# If repository key issues occur
wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# If package conflicts occur
sudo apt autoremove
sudo apt update
sudo apt upgrade
```

**Permission Denied for USB Device:**
```bash
# Check if device exists
ls -la /dev/ttyUSB*

# Fix permissions (temporary)
sudo chmod 666 /dev/ttyUSB0

# Fix permissions (permanent)
sudo usermod -a -G dialout $USER
# Logout and login again

# If udev rule doesn't work
sudo chmod 666 /dev/ttyUSB0
```

**Package Build Errors:**
```bash
# Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean rebuild
rm -rf build/ install/
colcon build --packages-select cartographer_rplidar_config

# If CMake errors occur
sudo apt install -y cmake build-essential
```

**Environment Issues:**
```bash
# If ROS2 commands not found
source /opt/ros/humble/setup.bash

# If package not found after build
source ~/ros2_ws/install/setup.bash

# Check ROS2 environment
printenv | grep ROS
```

### Runtime Issues

**RPLIDAR Not Detected:**
```bash
# Check USB connection
lsusb | grep -i silabs
ls -la /dev/ttyUSB*

# Test RPLIDAR manually
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000

# If no data received, try different baudrates
ros2 run sllidar_ros2 sllidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200
```

**Poor Map Quality:**
- **Move slower:** Rapid movements cause tracking loss
- **Better lighting:** RPLIDAR uses infrared, avoid bright sunlight
- **Loop closures:** Return to previously mapped areas
- **Avoid mirrors/glass:** Highly reflective surfaces cause issues
- **Check mounting:** RPLIDAR should be level and unobstructed

**RViz Not Showing Map:**
```bash
# Check if map topic is publishing
ros2 topic echo /map --once

# Check transforms
ros2 run tf2_tools view_frames

# Restart RViz if needed
pkill rviz2
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py
```

**Cartographer Crashes:**
```bash
# Check log output
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer.launch.py --screen

# If Lua configuration errors occur
cat ~/.ros/log/*/cartographer*/stdout.log

# Verify configuration file
ls -la ~/ros2_ws/src/cartographer_rplidar_config/config/rplidar_a3.lua
```

### Docker Issues

**X11 Display Problems:**
```bash
# Allow X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Alternative: use VNC or disable GUI
docker run -it --rm --device=/dev/ttyUSB0 rplidar-slam bash
```

**USB Device Not Accessible:**
```bash
# Use privileged mode
docker run -it --rm --privileged --device=/dev/ttyUSB0 rplidar-slam

# Or add specific permissions
docker run -it --rm --device=/dev/ttyUSB0:/dev/ttyUSB0:rwm rplidar-slam
```

### Getting Help

**Before Asking for Help:**
1. Check this troubleshooting section
2. Run the test script: `./test_slam.sh`
3. Check ROS2 logs: `ros2 log info`
4. Verify hardware connections

**Where to Get Support:**
- **GitHub Issues:** [Create an issue](https://github.com/peru00/rplidar-cartographer-slam/issues)
- **ROS2 Community:** [ROS Discourse](https://discourse.ros.org/)
- **RPLIDAR Support:** [Slamtec Documentation](https://www.slamtec.com/en/Support#rplidar-a3)

## 📊 Performance Tips

### For Best Mapping Results:
1. **Smooth motion:** Avoid sudden starts/stops
2. **Consistent speed:** 0.2-0.5 m/s walking speed
3. **Good features:** Map areas with walls, furniture, etc.
4. **Loop closures:** Essential for drift correction
5. **Adequate lighting:** Avoid direct sunlight interference

### Hardware Optimization:
- **Mount RPLIDAR level** and unobstructed
- **Stable mounting** to minimize vibrations
- **USB 2.0+ connection** for reliable data transfer
- **Adequate power supply** for consistent operation

## 🔄 Advanced Usage

### Custom Configuration
Edit `config/rplidar_a3.lua` to adjust:
- Scan matching parameters
- Loop closure sensitivity
- Pose graph optimization
- Range filtering

### Integration with Navigation
This SLAM system provides standard ROS2 topics:
- `/map` - Occupancy grid
- `/scan` - Laser scan data
- `/tf` - Transform tree
- `/odom` - Odometry estimates

## 📝 License

Licensed under the Apache License, Version 2.0. See the License for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📞 Support

- **Issues:** Create a GitHub issue
- **Documentation:** Check the troubleshooting section
- **Community:** ROS2 Discourse forums

## 🔗 Related Projects

- [Cartographer](https://github.com/cartographer-project/cartographer)
- [SLLIDAR ROS2](https://github.com/Slamtec/sllidar_ros2)
- [ROS2 Navigation](https://github.com/ros-planning/navigation2)

---

**Happy Mapping! 🗺️**
