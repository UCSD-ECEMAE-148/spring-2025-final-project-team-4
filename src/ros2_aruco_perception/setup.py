from setuptools import setup
import os

package_name = 'ros2_aruco_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            os.path.join(os.path.dirname(__file__), '..', 'launch',
                        'perception_and_planner.launch.py'),
        ]),

        ('share/' + package_name + '/calib_data', [
            'ros2_aruco_perception/calib_data/MultiMatrix.npz'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='popem',
    maintainer_email='popem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = ros2_aruco_perception.perception_node:main',
        ],
    },
)
