# Copyright 2019 Open Source Robotics Foundation, Inc.
# Author: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration('sim', default='false')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')

    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('shelfino_navigation'),
            'map',
            'turtle.yaml'))

    param_file_name = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('shelfino_navigation'),
            'config',
            'shelfino1.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('shelfino_navigation'),
        'rviz',
        'shelfino1_nav.rviz')

    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_name,
            description='Full path to param file to load'),

        DeclareLaunchArgument(name='sim', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),

        DeclareLaunchArgument(name='remote', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),

        DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': sim,
                'namespace': 'shelfino1',
                'use_namespace': 'True',
                'use_composition': 'False',
                'autostart': 'False',
                'use_respawn': 'True',
                'params_file': param_file_name}.items(),
            condition=UnlessCondition(remote),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino1',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': sim}],
            condition=UnlessCondition(headless),
            remappings=remappings,
            output='screen'),
    ])

    