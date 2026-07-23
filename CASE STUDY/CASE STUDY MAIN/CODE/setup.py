import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_follower'

def package_model_files(pkg_name):
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + pkg_name]),
        ('share/' + pkg_name, ['package.xml']),
        ('share/' + pkg_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + pkg_name + '/worlds', glob('worlds/*.world')),
        ('share/' + pkg_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
        ('share/' + pkg_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + pkg_name + '/params', glob('params/*.yaml')),
    ]
    # Recursively find all files in the models directory
    for root, dirs, files in os.walk('models'):
        if files:
            dest = os.path.join('share', pkg_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((dest, file_list))
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_model_files(package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ganeshna',
    maintainer_email='ganeshna@todo.todo',
    description='Autonomous person-following warehouse robot with Nav2, SLAM, YOLOv8 Re-ID and dynamic obstacle avoidance',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'perception_node      = robot_follower.perception_node:main',
            'behavior_tree_node   = robot_follower.behavior_tree_node:main',
            'manager_node         = robot_follower.manager_node:main',
            'bypass_follower_node = robot_follower.bypass_follower_node:main',
            'follower_node        = robot_follower.follower_node:main',
            'nav2_bridge_node     = robot_follower.nav2_bridge_node:main',
            'nav2_goal_client     = robot_follower.nav2_goal_client:main',
        ],
    },
)
