from setuptools import find_packages, setup
from glob import glob

package_name = 'droid_w_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='steve@local',
    description='ROS 2 wrapper for DROID-W (DROID-SLAM in the Wild).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'droid_w_node = droid_w_ros2.droid_w_node:main',
        ],
    },
)
