#!/bin/bash
# Create distribution package for RPLIDAR A3 Cartographer SLAM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="rplidar-a3-cartographer-slam"
VERSION="1.0.0"
DIST_DIR="$SCRIPT_DIR/dist"

echo "=== Creating RPLIDAR A3 Cartographer SLAM Distribution Package ==="
echo

# Create distribution directory
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"

# Create package directory
PACKAGE_DIR="$DIST_DIR/$PACKAGE_NAME-$VERSION"
mkdir -p "$PACKAGE_DIR"

echo "1. Copying package files..."
cp -r "$SCRIPT_DIR/config" "$PACKAGE_DIR/"
cp -r "$SCRIPT_DIR/launch" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/package.xml" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/CMakeLists.txt" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/install_rplidar_a3_slam.sh" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/test_slam.sh" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/save_map.sh" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/Dockerfile" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/docker-entrypoint.sh" "$PACKAGE_DIR/"
cp "$SCRIPT_DIR/SHARE_README.md" "$PACKAGE_DIR/README.md"

echo "2. Creating installation instructions..."
cat > "$PACKAGE_DIR/INSTALL.txt" << 'EOF'
RPLIDAR A3 Cartographer SLAM - Installation Instructions
======================================================

QUICK START:
1. Run: ./install_rplidar_a3_slam.sh
2. Connect RPLIDAR A3 to USB
3. Run: ./test_slam.sh
4. Start SLAM: ros2 launch cartographer_rplidar_config rplidar_a3_cartographer_rviz.launch.py

MANUAL INSTALLATION:
1. Ensure ROS2 Humble is installed
2. Copy cartographer_rplidar_config/ to your ROS2 workspace src/ folder
3. Run: colcon build --packages-select cartographer_rplidar_config
4. Source: source install/setup.bash

DOCKER INSTALLATION:
1. docker build -t rplidar-slam .
2. docker run -it --device=/dev/ttyUSB0 --privileged rplidar-slam

For detailed instructions, see README.md
EOF

echo "3. Creating version info..."
cat > "$PACKAGE_DIR/VERSION" << EOF
RPLIDAR A3 Cartographer SLAM
Version: $VERSION
Build Date: $(date)
ROS2 Distribution: Humble
Cartographer: Latest
RPLIDAR: A3 optimized
EOF

echo "4. Creating archive..."
cd "$DIST_DIR"
tar -czf "$PACKAGE_NAME-$VERSION.tar.gz" "$PACKAGE_NAME-$VERSION"
zip -r "$PACKAGE_NAME-$VERSION.zip" "$PACKAGE_NAME-$VERSION" > /dev/null

echo "5. Creating checksums..."
sha256sum "$PACKAGE_NAME-$VERSION.tar.gz" > "$PACKAGE_NAME-$VERSION.tar.gz.sha256"
sha256sum "$PACKAGE_NAME-$VERSION.zip" > "$PACKAGE_NAME-$VERSION.zip.sha256"

echo
echo "=== Distribution Package Created! ==="
echo
echo "Files created in: $DIST_DIR"
ls -la "$DIST_DIR"
echo
echo "Distribution options:"
echo "📦 Tarball:    $PACKAGE_NAME-$VERSION.tar.gz"
echo "📦 ZIP file:   $PACKAGE_NAME-$VERSION.zip"
echo "📁 Directory:  $PACKAGE_NAME-$VERSION/"
echo
echo "Share via:"
echo "• Upload to GitHub releases"
echo "• Send files directly"
echo "• Host on web server"
echo "• Docker Hub (docker build -t yourname/rplidar-slam .)"
echo
echo "Recipients can extract and run ./install_rplidar_a3_slam.sh"
