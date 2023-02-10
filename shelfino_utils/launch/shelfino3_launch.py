from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
ns = "shelfino3"
def generate_launch_description():
    my_package_dir = get_package_share_directory("shelfino_utils")
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            namespace=ns,
            remappings=remappings,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package='roadmap',
            executable='manager',
            arguments=['--ros-args --log-level debug'],
            namespace=ns,
        ),
        Node(
            package='roadmap',
            executable='driver',
            output='screen',
            namespace=ns,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(my_package_dir, 'config', 'testing.rviz')],
            output="log",
            remappings = remappings,
            namespace=ns,
            parameters=[{'use_sim_time': True}],
        )
    ])
