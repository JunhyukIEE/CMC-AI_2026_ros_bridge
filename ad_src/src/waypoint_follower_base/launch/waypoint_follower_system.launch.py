import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('waypoint_follower_base')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'waypoint_follower_params.yaml')

    # Waypoint follower base node
    waypoint_follower_base_node = Node(
        package='waypoint_follower_base',
        executable='waypoint_follower_base_node',
        name='waypoint_follower_base',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odom', '/odom'),
            ('/vehicle_state', '/vehicle_state'),
            ('/lateral_cmd', '/lateral_cmd'),
            ('/longitudinal_cmd', '/longitudinal_cmd'),
            ('/CtrlCmd', '/CtrlCmd'),
        ]
    )

    return LaunchDescription([
        waypoint_follower_base_node,
    ])
