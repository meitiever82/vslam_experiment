'''
Author: meitiever
Date: 2026-04-13 14:03:11
LastEditors: meitiever
LastEditTime: 2026-04-13 14:39:34
Description: content
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cuvslam_ros')
    params_file = os.path.join(pkg_dir, 'config', 'geoscan_stereo.yaml')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'cuvslam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('trajectory_output_path', default_value='',
                              description='Path to save TUM trajectory on shutdown'),

        Node(
            package='cuvslam_ros',
            executable='cuvslam_rosnode',
            name='cuvslam_ros',
            output='screen',
            parameters=[
                params_file,
                {'trajectory_output_path': LaunchConfiguration('trajectory_output_path')},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
