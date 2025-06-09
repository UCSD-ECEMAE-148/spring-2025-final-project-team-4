from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_camera',
            executable='oakd_executable',
            name='OAKD_node',
            output='screen',
        ),
        Node(
            package='ros2_aruco_perception',
            executable='perception_node',
            name='aruco_perception_node',
            output='screen',
        ),
    ])
