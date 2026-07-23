"""Start Nav2 with SLAM mapping or a previously saved warehouse map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def validate_static_map(context, *args, **kwargs):
    """Fail early instead of launching AMCL with a missing map file."""
    slam = LaunchConfiguration('slam').perform(context).strip().lower()
    if slam not in ('false', '0', 'no'):
        return []
    map_path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(map_path):
        raise RuntimeError(
            f'slam:=false requires an existing map YAML, got: {map_path or "<empty>"}')
    return []


def generate_launch_description():
    package_share = get_package_share_directory('robot_follower')
    nav2_share = get_package_share_directory('nav2_bringup')
    default_params = os.path.join(nav2_share, 'params', 'nav2_params.yaml')
    default_map = os.path.join(package_share, 'maps', 'warehouse2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Build a map with SLAM Toolbox when true'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Saved Nav2 map YAML used when slam is false'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameter file'),
        OpaqueFunction(function=validate_static_map),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                # nav2_bringup uses a Python expression and needs True/False,
                # while ROS users naturally pass lowercase true/false.
                'slam': PythonExpression([
                    "'", LaunchConfiguration('slam'),
                    "'.lower() in ['true', '1', 'yes']"]),
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])
