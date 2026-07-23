#!/usr/bin/env python3
"""
real_robot.launch.py — Launch file for REAL TurtleBot3 Waffle deployment.

Runs on the LAPTOP (off-board compute node):
  - perception_node: YOLOv8 + Re-ID + Kalman Filter
  - behavior_tree_node: 7-state BT + Nav2 client
  
The TurtleBot3 runs its OWN bringup separately on the robot's Raspberry Pi:
  ssh ubuntu@<robot_ip>
  ros2 launch turtlebot3_bringup robot.launch.py

Usage:
  export ROS_DOMAIN_ID=30
  export TURTLEBOT3_MODEL=waffle
  ros2 launch robot_follower real_robot.launch.py camera_topic:=/camera/image_raw
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters configurable from command line
    camera_topic    = LaunchConfiguration('camera_topic',    default='/camera/image_raw')
    scan_topic      = LaunchConfiguration('scan_topic',      default='/scan')
    camera_hfov     = LaunchConfiguration('camera_hfov',     default='1.047')  # 60 degrees
    yolo_model_path = LaunchConfiguration('yolo_model_path', default='yolov8n.pt')
    reid_threshold  = LaunchConfiguration('reid_threshold',  default='0.70')

    perception = Node(
        package='robot_follower',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # REAL robot: no sim time!
            'camera_topic': camera_topic,
            'scan_topic': scan_topic,
            'camera_hfov': camera_hfov,
            'yolo_model_path': yolo_model_path,
            'reid_threshold': reid_threshold,
        }],
    )
    
    behavior_tree = Node(
        package='robot_follower',
        executable='behavior_tree_node',
        name='behavior_tree_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # REAL robot: no sim time!
        }],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic',    default_value='/camera/image_raw'),
        DeclareLaunchArgument('scan_topic',      default_value='/scan'),
        DeclareLaunchArgument('camera_hfov',     default_value='1.047'),
        DeclareLaunchArgument('yolo_model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('reid_threshold',  default_value='0.70'),
        perception,
        behavior_tree,
    ])
