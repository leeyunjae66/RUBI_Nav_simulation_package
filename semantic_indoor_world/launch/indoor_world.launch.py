"""Gazebo Classic 11 with the semantic indoor world (no robot).

    ros2 launch semantic_indoor_world indoor_world.launch.py
    ros2 launch semantic_indoor_world indoor_world.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('semantic_indoor_world')
    gazebo_share = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'my_map.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        # The world is self-contained (no model:// includes); do not wait for the
        # online model database.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'server_required': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
