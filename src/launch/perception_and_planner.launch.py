from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam_driver',
            executable='usb_cam_node',
            name='usb_cam_node',
            output='screen',
        ),
        Node(
            package='ros2_aruco_perception',
            executable='perception_node',
            name='aruco_perception_node',
            output='screen',
        ),
        Node(
            package='ros2_task_planner',
            executable='task_planner_node',
            name='task_planner_node',
            output='screen',
        ),
    ])
