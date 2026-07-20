import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    waypoint_follower_pkg = get_package_share_directory('waypoint_follower_base')
    longitudinal_pkg = get_package_share_directory('morai_longitudinal_controller')
    lateral_pkg = get_package_share_directory('morai_lateral_controller')

    # Config file paths
    waypoint_config = os.path.join(waypoint_follower_pkg, 'config', 'waypoint_follower_params.yaml')
    pid_config = os.path.join(longitudinal_pkg, 'config', 'pid_params.yaml')
    pure_pursuit_config = os.path.join(lateral_pkg, 'config', 'pure_pursuit_params.yaml')

    # Waypoint follower base node
    waypoint_follower_base_node = Node(
        package='waypoint_follower_base',
        executable='waypoint_follower_base_node',
        name='waypoint_follower_base',
        output='screen',
        parameters=[waypoint_config],
        remappings=[
            ('/Ego/odom', '/odom'), # Remap /Ego/odom to /odom
        ],
    )

    # PID longitudinal controller node
    pid_controller_node = Node(
        package='morai_longitudinal_controller',
        executable='pid_controller_node',
        name='pid_controller',
        output='screen',
        parameters=[pid_config],
    )

    # Pure Pursuit lateral controller node
    pure_pursuit_controller_node = Node(
        package='morai_lateral_controller',
        executable='pure_pursuit_controller_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[pure_pursuit_config],
    )

    return LaunchDescription([
        waypoint_follower_base_node,
        pid_controller_node,
        pure_pursuit_controller_node,
    ])