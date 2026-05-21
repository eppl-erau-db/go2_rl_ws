import os
from glob import glob

from setuptools import setup

package_name = 'go2_bag_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='RViz marker visualization for Go2 dated rosbag playback.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'marker_node = go2_bag_visualizer.marker_node:main',
            'find_dated_bags = go2_bag_visualizer.bag_finder:main',
            'plot_bags_3d = go2_bag_visualizer.matplotlib_bag_viewer:main',
        ],
    },
)
