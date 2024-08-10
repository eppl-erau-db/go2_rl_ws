from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_lidar_decoder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Unitree Go2 Lidar Decoder for RL inputs.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_height_map = unitree_ros2_python.go2_height_map:main',
            'go2_height_map_visualizer = unitree_ros2_python.go2_height_map_visualizer:main',
            'go2_depth_map = unitree_ros2_python.go2_depth_map:main',
            'go2_depth_map_visualizer = unitree_ros2_python.go2_depth_map_visualizer:main'
        ],
    },
)


