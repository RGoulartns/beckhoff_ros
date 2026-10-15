#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

params = os.path.join(
    get_package_share_directory('beckhoff_ros'),
    'config',
    'params.json'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=params,
            description='JSON file describing the PLC: connection, structures and variables'
        ),
        Node(
            package='beckhoff_ros',
            executable='beckhoff_ros_node',
            name='beckhoff_ros_node',
            parameters=[{'config_file': LaunchConfiguration('config_file')}],
            output='screen'
        )
    ])
