from setuptools import setup
import os
from glob import glob

package_name = 'unitree_ros2_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('share/models/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Unitree ROS 2 Python package',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_base_vel = unitree_ros2_python.go2_base_vel:main',
            'go2_joint_pos_vel = unitree_ros2_python.go2_joint_pos_vel:main',
            'go2_projected_gravity = unitree_ros2_python.go2_projected_gravity:main',
            'go2_velocity_commands = unitree_ros2_python.go2_velocity_commands:main',
            'go2_rl_actions = unitree_ros2_python.go2_rl_actions:main'
        ],
    },
)
