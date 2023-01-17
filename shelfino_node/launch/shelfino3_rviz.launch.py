#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/goal_pose', 'goal_pose'),
                  ('/clicked_point', 'clicked_point'),
                  ('/initialpose', 'initialpose')]

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino3',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings,
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory("shelfino_node"),
                                   'rviz', 'shelfino3.rviz')]
        )
    ])
