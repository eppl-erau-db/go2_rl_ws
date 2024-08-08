from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
         glob('share/models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabearod2@gmail.com',
    description='Package to deploy RL navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_pose_command = rl_navigation.go2_pose_command:main',
            'go2_rl_nav_actions_onnx = rl_navigation.go2_rl_nav_actions_onnx:main',
            'go2_rl_nav_actions_jit = rl_navigation.go2_rl_nav_actions_jit:main'
        ],
    },
)
