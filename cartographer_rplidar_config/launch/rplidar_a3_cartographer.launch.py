#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get the launch directory
    pkg_cartographer_rplidar_config = FindPackageShare(package='cartographer_rplidar_config').find('cartographer_rplidar_config')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # RPLIDAR A3 node
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        parameters=[{
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,  # RPLIDAR A3 baudrate
            'inverted': False,
            'auto_standby': True
        }],
        output='screen')
    
    # Robot state publisher for TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': """
                <?xml version="1.0"?>
                <robot name="rplidar_robot">
                  <link name="base_link">
                    <visual>
                      <geometry>
                        <box size="0.1 0.1 0.1"/>
                      </geometry>
                      <material name="red">
                        <color rgba="1.0 0.0 0.0 1.0"/>
                      </material>
                    </visual>
                  </link>
                  <link name="laser">
                    <visual>
                      <geometry>
                        <cylinder radius="0.05" length="0.04"/>
                      </geometry>
                      <material name="black">
                        <color rgba="0.0 0.0 0.0 1.0"/>
                      </material>
                    </visual>
                  </link>
                  <joint name="laser_joint" type="fixed">
                    <parent link="base_link"/>
                    <child link="laser"/>
                    <origin xyz="0 0 0.02" rpy="0 0 0"/>
                  </joint>
                </robot>
            """,
            'use_sim_time': use_sim_time
        }],
        output='screen')
    
    # Cartographer node
    cartographer_config_dir = os.path.join(pkg_cartographer_rplidar_config, 'config')
    configuration_basename = 'rplidar_a3.lua'
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename])
    
    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the actions to the launch description
    ld.add_action(rplidar_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    
    return ld
