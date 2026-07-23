"""Launch the warehouse, a GPU-rendered Gazebo client, and an optimized Waffle."""

import os
import tempfile
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Pre-set Gazebo model path at python environment level to ensure subprocess inheritance
pkg_share = get_package_share_directory('robot_follower')
wm_path = os.path.join(pkg_share, 'models')
tb_share = get_package_share_directory('turtlebot3_gazebo')
tb_models = os.path.join(tb_share, 'models')
gmp_env = os.environ.get('GAZEBO_MODEL_PATH', '')
if gmp_env:
    os.environ['GAZEBO_MODEL_PATH'] = f"{wm_path}:{tb_models}:{gmp_env}"
else:
    os.environ['GAZEBO_MODEL_PATH'] = f"{wm_path}:{tb_models}"


def perception_camera_sdf(source, model, width, height, rate):
    """Create a lower-cost camera SDF without modifying TurtleBot's install."""
    tree = ElementTree.parse(source)
    sensor = next((item for item in tree.iter('sensor')
                   if item.get('name') == 'camera' and item.get('type') == 'camera'), None)
    if sensor is None:
        return source
    sensor.find('update_rate').text = str(rate)
    sensor.find('visualize').text = 'false'
    image = sensor.find('camera/image')
    image.find('width').text = str(width)
    image.find('height').text = str(height)
    output = os.path.join(tempfile.gettempdir(),
                          f'turtlebot3_{model}_{width}x{height}_{rate}hz_camera.sdf')
    # spawn_entity.py reads SDF as text; lxml rejects a text XML declaration.
    tree.write(output, encoding='unicode')
    return output


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    if model not in ('burger', 'waffle', 'waffle_pi'):
        raise RuntimeError('model must be burger, waffle, or waffle_pi')

    gazebo_share = get_package_share_directory('gazebo_ros')
    turtlebot_share = get_package_share_directory('turtlebot3_gazebo')

    world_path = LaunchConfiguration('world').perform(context)
    warehouse_models = LaunchConfiguration('warehouse_models').perform(context)
    turtlebot_models = os.path.join(turtlebot_share, 'models')
    robot_sdf = perception_camera_sdf(
        os.path.join(turtlebot_share, 'models', f'turtlebot3_{model}', 'model.sdf'),
        model,
        int(LaunchConfiguration('camera_width').perform(context)),
        int(LaunchConfiguration('camera_height').perform(context)),
        int(LaunchConfiguration('camera_rate').perform(context)),
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': LaunchConfiguration('verbose')}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', f'turtlebot3_{model}',
            '-file', robot_sdf,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.05',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', f'{warehouse_models}:{turtlebot_models}'),
        gazebo_server,
        # Wait until the large warehouse scene exists before opening gzclient.
        TimerAction(period=LaunchConfiguration('gui_delay'), actions=[gazebo_client]),
        robot_state_publisher,
        spawn_robot,
    ]


def generate_launch_description():
    package_share = get_package_share_directory('robot_follower')
    default_world = os.path.join(package_share, 'worlds', 'warehouse2_dynamic.world')
    default_models = os.path.join(package_share, 'models')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Warehouse world file'),
        DeclareLaunchArgument('warehouse_models', default_value=default_models, description='Warehouse model directory'),
        DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model'),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot start X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot start Y'),
        DeclareLaunchArgument('yaw', default_value='1.5708', description='Robot start yaw'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo client'),
        DeclareLaunchArgument('verbose', default_value='false', description='Verbose Gazebo output'),
        DeclareLaunchArgument('camera_width', default_value='640', description='Waffle RGB camera width'),
        DeclareLaunchArgument('camera_height', default_value='480', description='Waffle RGB camera height'),
        DeclareLaunchArgument('camera_rate', default_value='20', description='Waffle RGB camera FPS'),
        DeclareLaunchArgument('gui_delay', default_value='12.0', description='Seconds to wait before opening Gazebo GUI'),
        DeclareLaunchArgument(
            'use_nvidia', default_value='false',
            description='Use NVIDIA PRIME for Gazebo rendering (disabled by default on Wayland)'),
        # Gazebo Classic is more reliable through XWayland on this laptop.
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable(
            '__NV_PRIME_RENDER_OFFLOAD', '1',
            condition=IfCondition(LaunchConfiguration('use_nvidia'))),
        SetEnvironmentVariable(
            '__GLX_VENDOR_LIBRARY_NAME', 'nvidia',
            condition=IfCondition(LaunchConfiguration('use_nvidia'))),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('model')),
        OpaqueFunction(function=launch_setup),
    ])
