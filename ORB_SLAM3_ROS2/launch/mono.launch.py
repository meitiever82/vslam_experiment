from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('orbslam3_ros2')

    vocabulary_file_arg = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=PathJoinSubstitution([pkg_share, 'vocabulary', 'ORBvoc.txt']),
    )

    # Reuses the mono-inertial GeoScan yaml; pure-mono sensor mode just
    # ignores the IMU.* fields.
    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'monocular-inertial', 'GeoScan_fisheye.yaml']),
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/left_camera/image',
    )

    orb_slam3_node = Node(
        package='orbslam3_ros2',
        executable='mono',
        name='orb_slam3_mono',
        output='screen',
        arguments=[
            LaunchConfiguration('vocabulary_file'),
            LaunchConfiguration('settings_file'),
        ],
        remappings=[
            ('camera', LaunchConfiguration('image_topic')),
        ],
    )

    return LaunchDescription([
        vocabulary_file_arg,
        settings_file_arg,
        image_topic_arg,
        orb_slam3_node,
    ])
