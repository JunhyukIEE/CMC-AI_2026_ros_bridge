from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="morai_bridge",
                executable="morai_sensor_receiver_node",
                output="screen",
            ),
            Node(
                package="morai_bridge",
                executable="navsat_to_gnss_node",
                output="screen",
            ),
        ]
    )
