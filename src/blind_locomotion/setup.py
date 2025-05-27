from setuptools import setup
import os
from glob import glob

package_name = 'blind_locomotion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
         glob('share/models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Python package for blind RL locomotion.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_actions = blind_locomotion.rl_actions:main',
            'controller_commands = blind_locomotion.controller_commands:main'
        ],
    },
)
