#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

params = os.path.join(
  get_package_share_directory('beckhoff_ros'),
  'config',
  'params.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beckhoff_ros',
            executable='beckhoff_ros_node',
            name='beckhoff_ros_node',
            parameters=[params],
            output='screen'
        )
    ])