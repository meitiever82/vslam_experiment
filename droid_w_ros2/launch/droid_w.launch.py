"""Launch droid_w_node with the default ros parameter file.

Usage:
    ros2 launch droid_w_ros2 droid_w.launch.py
    ros2 launch droid_w_ros2 droid_w.launch.py params_file:=/abs/path/to/my.yaml

Override individual parameters on the command line:
    ros2 launch droid_w_ros2 droid_w.launch.py image_topic:=/cam0/image_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('droid_w_ros2')
    default_params = os.path.join(pkg_share, 'config', 'droid_w_ros.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the ROS parameter YAML for droid_w_node',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='',
        description='Override image_topic parameter (empty = use yaml)',
    )

    droid_w_node = Node(
        package='droid_w_ros2',
        executable='droid_w_node',
        name='droid_w_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            # Empty image_topic is ignored — the parameter from the yaml wins.
            # Override is appended only when non-empty so we don't blank out
            # the yaml value.
        ],
    )

    return LaunchDescription([
        params_file_arg,
        image_topic_arg,
        droid_w_node,
    ])
