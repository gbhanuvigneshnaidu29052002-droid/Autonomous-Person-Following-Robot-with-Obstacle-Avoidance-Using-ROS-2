"""Launch the perception node for the Waffle Pi camera + LiDAR."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory('person_reid_perception'),
        'config',
        'perception.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=cfg,
            description='Path to the perception.yaml config file.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Input RGB image topic.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Input LiDAR LaserScan topic.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated clock (Gazebo).',
        ),

        Node(
            package='person_reid_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('image_raw', LaunchConfiguration('image_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        ),
    ])
