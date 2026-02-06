from setuptools import setup
import os
from glob import glob

package_name = 'blind_locomotion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'),
        glob('share/models/*')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Python package for blind RL locomotion.',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_actions = blind_locomotion.rl_actions:main',
            'rl_reach_actions = blind_locomotion.rl_reach_actions:main',
            'controller_commands = blind_locomotion.controller_commands:main',
            'test_action_publisher.py = blind_locomotion.test_action_publisher:main',
            'keyboard_buttons = blind_locomotion.keyboard_buttons:main',
            'fake_lowstate_publisher.py = blind_locomotion.fake_lowstate_publisher:main',
            'lowcmd_decoder.py = blind_locomotion.lowcmd_decoder:main',
            'lowstate_monitor.py = blind_locomotion.lowstate_monitor:main',
        ],
    },
)
