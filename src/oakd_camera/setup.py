from setuptools import setup
import os
from glob import glob

package_name = 'oakd_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', [
            os.path.join(os.path.dirname(__file__), '..', 'launch',
                        'OAKD_perception_test.launch.py'),
        ]),
    ],
    install_requires=['setuptools',
        'depthai',    
        'opencv-python',  
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'oakd_executable=oakd_camera.OAKD_RAW:main',
        'listen_executable=oakd_camera.OAKD_listen:main'
        ],
    },
)
