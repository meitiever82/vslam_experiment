'''
Author: meitiever
Date: 2026-04-13 14:03:15
LastEditors: meitiever
LastEditTime: 2026-04-15 12:21:38
Description: content
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('orbslam3_ros2')

    # 声明 launch 参数
    vocabulary_file_arg = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=PathJoinSubstitution([pkg_share, 'vocabulary', 'ORBvoc.txt']),
        description='Path to ORB vocabulary file'
    )

    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'monocular-inertial', 'GeoScan_fisheye.yaml']),
        description='Path to settings file'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/left_camera/image',
        description='Image topic name'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/handsfree/imu',
        description='IMU topic name'
    )

    do_equalize_arg = DeclareLaunchArgument(
        'do_equalize',
        default_value='true',
        description='Enable histogram equalization (CLAHE)'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable Pangolin visualization'
    )

    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='false',
        description='Use compressed image topics (true) or raw image topics (false)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 with a preconfigured view'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'mono_inertial.rviz']),
        description='Path to RViz config file'
    )

    # ORB-SLAM3 单目惯性节点
    orb_slam3_node = Node(
        package='orbslam3_ros2',
        executable='mono-inertial',
        name='orb_slam3_mono_inertial',
        output='screen',
        arguments=[
            LaunchConfiguration('vocabulary_file'),
            LaunchConfiguration('settings_file'),
            LaunchConfiguration('do_equalize'),
            LaunchConfiguration('enable_visualization'),
            LaunchConfiguration('use_compressed'),
        ],
        remappings=[
            ('camera', LaunchConfiguration('image_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        vocabulary_file_arg,
        settings_file_arg,
        image_topic_arg,
        imu_topic_arg,
        do_equalize_arg,
        enable_visualization_arg,
        use_compressed_arg,
        use_rviz_arg,
        rviz_config_arg,
        orb_slam3_node,
        rviz_node,
    ])
