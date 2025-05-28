from setuptools import setup

package_name = 'ros2_task_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ros2_task_planner']),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/task_planner_node']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Task planner node for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_planner_node = ros2_task_planner.task_planner_node:main',
        ],
    },
)
