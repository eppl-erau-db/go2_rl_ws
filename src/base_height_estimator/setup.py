from setuptools import setup

package_name = 'base_height_estimator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Kinematic base height estimator for Unitree Go2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_height_node = base_height_estimator.base_height_node:main',
        ],
    },
)
