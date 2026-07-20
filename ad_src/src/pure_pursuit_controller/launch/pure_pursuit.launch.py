import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('pure_pursuit_controller'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node', # The C++ executable name
            name='pure_pursuit_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        ),
    ])
