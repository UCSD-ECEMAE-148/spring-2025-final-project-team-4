from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_aruco_perception',
            executable='perception_node',
            name='aruco_perception_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}],  # Add any parameters needed for your node
            remappings=[
                # Add any topic remappings if necessary
            ],
        ),
    ])