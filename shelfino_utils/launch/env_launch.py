from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

def generate_launch_description():
    my_package_dir = get_package_share_directory("shelfino_utils")
    return LaunchDescription([
        Node(
            package='send_borders',
            executable='send_borders',
            name='borders_node'
        ),
        Node(
            package='send_obstacles',
            executable='send_obstacles',
            name='obstacles_node'
        ),
        Node(
            package='send_gates',
            executable='send_gates',
            name='gates_node'
        ),
        Node(
            package='dubins_calculator',
            executable='dubins_calculator',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(my_package_dir, 'config', 'testing.rviz')],
            remappings = remappings,
            namespace='shelfinoG',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    ])
