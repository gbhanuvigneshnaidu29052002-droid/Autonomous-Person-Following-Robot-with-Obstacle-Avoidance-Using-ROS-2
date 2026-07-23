"""Desk/Gazebo smoke test: dummy camera + perception node.

Use this launch file when there is no real camera available. It starts a
synthetic camera publisher and the perception node, both wired to the
default topics the perception node expects.

On the real Waffle Pi, the camera is provided by the existing
``realsense2_camera`` launch on the Pi; use ``perception.launch.py``
instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def _maybe_start_dummy(context, *args, **kwargs):
    """Conditionally include the dummy camera publisher."""
    use_dummy = LaunchConfiguration('use_dummy_camera').perform(context).lower() == 'true'
    image_topic = LaunchConfiguration('image_topic').perform(context)
    if not use_dummy:
        return []
    return [
        Node(
            package='person_reid_perception',
            executable='dummy_camera_publisher',
            name='dummy_camera_publisher',
            output='screen',
            parameters=[{
                'publish_topic': image_topic,
                'image_width': 640,
                'image_height': 480,
                'fps': 15.0,
                'frame_id': 'camera_optical_frame',
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    perception_launch = os.path.join(
        get_package_share_directory('person_reid_perception'),
        'launch',
        'perception.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_dummy_camera', default_value='true'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        OpaqueFunction(function=_maybe_start_dummy),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'image_topic': LaunchConfiguration('image_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
