import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('morai_lateral_controller')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'pure_pursuit_params.yaml')

    # Pure Pursuit controller node
    pure_pursuit_controller_node = Node(
        package='morai_lateral_controller',
        executable='pure_pursuit_controller_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/vehicle_state', '/vehicle_state'),
            ('/lateral_cmd', '/lateral_cmd'),
        ]
    )

    return LaunchDescription([
        pure_pursuit_controller_node,
    ])
