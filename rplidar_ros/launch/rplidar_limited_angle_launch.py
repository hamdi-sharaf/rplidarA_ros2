#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('rplidar_ros')
    
    # Declare launch arguments
    channel_type_arg = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Specifying channel type of lidar (serial, tcp, udp)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Specifying usb port baudrate to connected lidar'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Specifying frame_id of lidar'
    )
    
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='true',
        description='Specifying whether or not to invert scan data'
    )
    
    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='',
        description='Specifying scan mode of lidar'
    )
    
    # New angle limit arguments
    angle_min_limit_arg = DeclareLaunchArgument(
        'angle_min_limit',
        default_value='120.0',
        description='Minimum angle limit in degrees (e.g., 120.0)'
    )
    
    angle_max_limit_arg = DeclareLaunchArgument(
        'angle_max_limit',
        default_value='240.0',
        description='Maximum angle limit in degrees (e.g., 240.0)'
    )
    
    # TF transform arguments (base_link to laser_frame)
    tf_x_arg = DeclareLaunchArgument(
        'tf_x',
        default_value='0.1',
        description='X position of laser relative to base_link (meters)'
    )
    
    tf_y_arg = DeclareLaunchArgument(
        'tf_y',
        default_value='0.0',
        description='Y position of laser relative to base_link (meters)'
    )
    
    tf_z_arg = DeclareLaunchArgument(
        'tf_z',
        default_value='0.2',
        description='Z position of laser relative to base_link (meters)'
    )
    
    tf_roll_arg = DeclareLaunchArgument(
        'tf_roll',
        default_value='0.0',
        description='Roll rotation of laser relative to base_link (radians)'
    )
    
    tf_pitch_arg = DeclareLaunchArgument(
        'tf_pitch',
        default_value='0.0',
        description='Pitch rotation of laser relative to base_link (radians)'
    )
    
    tf_yaw_arg = DeclareLaunchArgument(
        'tf_yaw',
        default_value='0.0',
        description='Yaw rotation of laser relative to base_link (radians)'
    )
    
    # Static TF broadcaster node
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', LaunchConfiguration('tf_x'),
            '--y', LaunchConfiguration('tf_y'),
            '--z', LaunchConfiguration('tf_z'),
            '--roll', LaunchConfiguration('tf_roll'),
            '--pitch', LaunchConfiguration('tf_pitch'),
            '--yaw', LaunchConfiguration('tf_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('frame_id')
        ]
    )
    
    # Create the node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': LaunchConfiguration('channel_type'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': LaunchConfiguration('inverted'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode'),
            'angle_min_limit': LaunchConfiguration('angle_min_limit'),
            'angle_max_limit': LaunchConfiguration('angle_max_limit'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        channel_type_arg,
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        angle_min_limit_arg,
        angle_max_limit_arg,
        tf_x_arg,
        tf_y_arg,
        tf_z_arg,
        tf_roll_arg,
        tf_pitch_arg,
        tf_yaw_arg,
        static_tf_node,
        rplidar_node
    ])
