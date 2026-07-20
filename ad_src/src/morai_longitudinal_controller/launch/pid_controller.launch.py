import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('morai_longitudinal_controller')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'pid_params.yaml')

    # PID controller node
    pid_controller_node = Node(
        package='morai_longitudinal_controller',
        executable='pid_controller_node',
        name='pid_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/vehicle_state', '/vehicle_state'),
            ('/longitudinal_cmd', '/longitudinal_cmd'),
        ]
    )

    return LaunchDescription([
        pid_controller_node,
    ])
