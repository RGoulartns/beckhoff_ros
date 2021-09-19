from launch import LaunchDescription
from launch_ros.actions import Node
import os


params = os.path.join(
  'INSERT_PATH/src/beckhoff_ros',
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