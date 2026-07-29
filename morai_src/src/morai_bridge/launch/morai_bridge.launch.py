import os
from math import radians

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def static_tf(parent, child, xyz, rpy_deg):
    roll, pitch, yaw = (radians(angle) for angle in rpy_deg)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
    )


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
            static_tf(
                "base_link", "velodyne_top",
                (1.25, 0.0, 1.5), (0.0, 0.0, 0.0),
            ),
            static_tf("base_link", "gnss_link", (0.0, 0.0, 1.2), (0.0, 0.0, 0.0)),
            static_tf("base_link", "imu", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            static_tf(
                "base_link", "front_camera/camera_link",
                (1.9, 0.0, 1.2), (0.0, 2.0, 0.0),
            ),
            static_tf(
                "front_camera/camera_link", "front_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            static_tf(
                "base_link", "left_camera/camera_link",
                (1.15, 0.65, 1.2), (0.0, 10.0, 70.0),
            ),
            static_tf(
                "left_camera/camera_link", "left_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            static_tf(
                "base_link", "right_camera/camera_link",
                (1.15, -0.65, 1.2), (0.0, 10.0, -70.0),
            ),
            static_tf(
                "right_camera/camera_link", "right_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            static_tf(
                "base_link", "traffic_light_left_camera/camera_link",
                (1.8, 0.0, 1.3), (0.0, -15.0, 0.0),
            ),
            static_tf(
                "traffic_light_left_camera/camera_link",
                "traffic_light_left_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
        ]
    )
