#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world_file_name = 'hexagon.world'
    world = os.path.join(get_package_share_directory('shelfino_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    rviz_config1 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino1.rviz')
    rviz_config2 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino2.rviz')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                description='Flag to enable gazebo visualization'),

        DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
                                description='Flag to enable rviz visualization'),

        DeclareLaunchArgument('model', default_value=[os.path.join(gazebo_models_path, 'shelfino'),'/model.sdf']),
        LogInfo(msg=LaunchConfiguration('model')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui)
        ),
    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', 'shelfino1',
                       '-robot_namespace', 'shelfino1',
                       '-x', '0',
                       '-y', '1']
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', 'shelfino2',
                       '-robot_namespace', 'shelfino2',
                       '-x', '0',
                       '-y', '0']
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', 'shelfino3',
                       '-robot_namespace', 'shelfino3',
                       '-x', '0',
                       '-y', '-1']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': 'shelfino1'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': 'shelfino2'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': 'shelfino3'}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino1',
            arguments=['-d', rviz_config1],
            condition=IfCondition(rviz),
            remappings=remappings
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino2',
            arguments=['-d', rviz_config1],
            condition=IfCondition(rviz),
            remappings=remappings
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino3',
            arguments=['-d', rviz_config1],
            condition=IfCondition(rviz),
            remappings=remappings
        ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace='shelfino1',
            remappings=[
            ('/tf', 'tf')],
            ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace='shelfino2',
            remappings=[
            ('/tf', 'tf')],
            ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace='shelfino3',
            remappings=[
            ('/tf', 'tf')],
            ),

        Node(
            package='send_obstacles',
            executable='send_obstacles'
        ),

        Node(
            package='send_borders',
            executable='send_borders'
        ),

        Node(
            package='send_gates',
            executable='send_gates'
        ),
    ])
