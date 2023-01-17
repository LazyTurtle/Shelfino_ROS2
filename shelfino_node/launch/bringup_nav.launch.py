#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    robot_id = LaunchConfiguration('robot_id', default='shelfino404')
    remote = LaunchConfiguration('use_rviz', default='false')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('shelfino_navigation'), 'launch')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='shelfino404',
            description='Shelfino ID'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use Rviz defaults to false'),

        Node(
            package='shelfino_node',
            namespace=robot_id,
            executable='shelfino_node',
            remappings=remappings
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_id}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/', robot_id, '_nav.launch.py']),
            launch_arguments={'remote': remote}.items()
        ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace=robot_id,
            remappings=[
            ('/tf', 'tf')
        ]),
    ])
