import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("morai_bridge"), "config", "morai_bridge.yaml"
    )
    camera_launch = os.path.join(
        get_package_share_directory("udp_camera_receiver"),
        "launch",
        "udp_camera_receiver.launch.py",
    )
    return LaunchDescription(
        [
            Node(
                package="morai_bridge",
                executable="morai_sensor_receiver_node",
                output="screen",
                parameters=[params],
                remappings=[("/gps/fix", "/sensing/gnss/ublox/nav_sat_fix")],
            ),
            Node(
                package="morai_bridge",
                executable="navsat_to_gnss_node",
                output="screen",
                remappings=[("/gps/fix", "/sensing/gnss/ublox/nav_sat_fix")],
            ),
            Node(
                package="morai_bridge",
                executable="morai_receiver_node",
                output="screen",
            ),
            Node(
                package="morai_bridge",
                executable="morai_sender_node",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
            ),
        ]
    )
