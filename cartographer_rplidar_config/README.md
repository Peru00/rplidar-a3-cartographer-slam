# RPLIDAR A3 Cartographer SLAM - Complete Setup Package

A ready-to-use 2D SLAM system using **RPLIDAR A3** with **Google Cartographer** on **ROS2 Humble**. This package provides a complete configuration optimized for RPLIDAR A3 without requiring IMU or wheel odometry.

## 🚀 Quick Start

### Method 1: Automated Installation (Recommended)

1. **Download and run the installation script:**
```bash
curl -fsSL https://raw.githubusercontent.com/peru00/rplidar-cartographer-slam/main/install_rplidar_a3_slam.sh | bash
```

### Method 2: Manual Installation

1. **Clone this repository:**
```bash
git clone https://github.com/peru00/rplidar-cartographer-slam.git
cd rplidar-cartographer-slam
./install_rplidar_a3_slam.sh
```

### Method 3: Using Existing ROS2 Workspace

1. **Copy the package to your workspace:**
```bash
cp -r cartographer_rplidar_config /path/to/your/ros2_ws/src/
cd /path/to/your/ros2_ws
colcon build --packages-select cartographer_rplidar_config
source install/setup.bash
```

### Method 4: Docker (Cross-platform)

```bash
# Build the Docker image
docker build -t rplidar-slam .

# Run with USB device access
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  rplidar-slam

# Inside container, run SLAM
ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py
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

### RPLIDAR Not Detected
```bash
# Check USB connection
ls -la /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Poor Map Quality
- **Move slower:** Rapid movements cause tracking loss
- **Better lighting:** RPLIDAR uses infrared, avoid bright sunlight
- **Loop closures:** Return to previously mapped areas
- **Avoid mirrors/glass:** Highly reflective surfaces cause issues

### Build Issues
```bash
# Update dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean rebuild
rm -rf build/ install/
colcon build --packages-select cartographer_rplidar_config
```

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
