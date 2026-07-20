import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('waypoint_recorder'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_recorder',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
