"""Setup script for the person_reid_perception ROS 2 ament_python package."""

import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'person_reid_perception'

# The tracker is a sibling project. The path is configurable via an env var so
# the same setup.py works on a dev box and on the Pi. The default points to
# the conventional location on the Waffle Pi's user account.
_TRACKER_PATH = os.environ.get(
    'PERSON_REID_TRACKER_PATH',
    '/home/ubuntu/person_reid_tracker',
)

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        (os.path.join('share', PACKAGE_NAME), ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'person_reid_tracker @ file://' + _TRACKER_PATH,
    ],
    zip_safe=False,
    maintainer='Maintainer',
    maintainer_email='dev@example.com',
    description='ROS 2 perception wrapper for person_reid_tracker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = '
                'person_reid_perception.perception_node:main',
            'dummy_camera_publisher = '
                'person_reid_perception.dummy_camera_publisher:main',
        ],
    },
)
