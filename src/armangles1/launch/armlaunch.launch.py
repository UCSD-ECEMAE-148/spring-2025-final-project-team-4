from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='armangles',
            executable='sendangles_executable',
            output='screen'),
    ])
