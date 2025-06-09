from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_camera',      # Your package name
            executable='oakd_executable',  # Entry point name from setup.py
            output='screen',            # Log output to terminal
        ),
        Node(
            package='oakd_camera',
            executable='listen_executable',
            output='screen',
        ),
    ])
