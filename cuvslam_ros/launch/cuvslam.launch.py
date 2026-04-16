import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cuvslam_ros')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'cuvslam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),

        # ---- Launch arguments ----
        DeclareLaunchArgument('left_image_topic',
            default_value='/camera/left/image_raw',
            description='Left camera image topic'),

        DeclareLaunchArgument('right_image_topic',
            default_value='/camera/right/image_raw',
            description='Right camera image topic'),

        DeclareLaunchArgument('imu_topic',
            default_value='',
            description='IMU topic (empty to disable)'),

        DeclareLaunchArgument('camera_info_topic',
            default_value='',
            description='CameraInfo topic (empty if using params)'),

        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('fx', default_value='554.0'),
        DeclareLaunchArgument('fy', default_value='554.0'),
        DeclareLaunchArgument('cx', default_value='640.0'),
        DeclareLaunchArgument('cy', default_value='360.0'),
        DeclareLaunchArgument('baseline', default_value='0.12'),

        DeclareLaunchArgument('odometry_mode',
            default_value='stereo',
            description='stereo or inertial'),

        DeclareLaunchArgument('enable_slam',
            default_value='false',
            description='Enable SLAM loop closure'),

        DeclareLaunchArgument('enable_landmarks',
            default_value='false',
            description='Publish landmarks for RViz'),

        # ---- Node ----
        Node(
            package='cuvslam_ros',
            executable='cuvslam_rosnode',
            name='cuvslam_ros',
            output='screen',
            parameters=[{
                'left_image_topic': LaunchConfiguration('left_image_topic'),
                'right_image_topic': LaunchConfiguration('right_image_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'fx': LaunchConfiguration('fx'),
                'fy': LaunchConfiguration('fy'),
                'cx': LaunchConfiguration('cx'),
                'cy': LaunchConfiguration('cy'),
                'baseline': LaunchConfiguration('baseline'),
                'odometry_mode': LaunchConfiguration('odometry_mode'),
                'enable_slam': LaunchConfiguration('enable_slam'),
                'enable_landmarks': LaunchConfiguration('enable_landmarks'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
