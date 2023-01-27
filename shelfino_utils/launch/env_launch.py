from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    ])
