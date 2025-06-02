FROM osrf/ros:humble-desktop

# Install system dependencies
RUN apt-get update && apt-get install -y \
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
    liblua5.3-dev \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /opt/ros2_ws
RUN mkdir -p src

# Clone required packages
WORKDIR /opt/ros2_ws/src
RUN git clone https://github.com/cartographer-project/cartographer.git && \
    git clone https://github.com/cartographer-project/cartographer_ros.git && \
    git clone https://github.com/Slamtec/sllidar_ros2.git

# Copy our custom package
COPY cartographer_rplidar_config/ ./cartographer_rplidar_config/

# Install dependencies
WORKDIR /opt/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select sllidar_ros2 cartographer_rplidar_config --symlink-install"

# Set up entry point
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

# Create user for USB access
RUN groupadd -r slamuser && useradd -r -g slamuser slamuser
RUN usermod -a -G dialout slamuser

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
