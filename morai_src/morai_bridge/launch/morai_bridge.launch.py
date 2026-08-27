from math import radians
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


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
    with open(params, encoding='utf-8') as config_file:
        wheel_radius = float(
            yaml.safe_load(config_file)['morai_bridge_launch']['ros__parameters'][
                'wheel_radius'
            ]
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
                parameters=[params],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
            ),
            static_tf(
                "base_link", "velodyne_top",
                (1.6, 0.0, 1.63 + wheel_radius), (0.0, 0.0, 0.0),
            ),
            static_tf(
                "base_link", "gnss_link",
                (0.0, 0.0, 1.2 + wheel_radius), (0.0, 0.0, 0.0),
            ),
            static_tf(
                "base_link", "imu",
                (0.0, 0.0, wheel_radius), (0.0, 0.0, 0.0),
            ),
            static_tf(
                "base_link", "front_camera/camera_link",
                (1.9, 0.0, 1.2 + wheel_radius), (0.0, 2.0, 0.0),
            ),
            static_tf(
                "front_camera/camera_link", "front_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            static_tf(
                "base_link", "left_camera/camera_link",
                (1.15, 0.65, 1.2 + wheel_radius), (0.0, 10.0, 70.0),
            ),
            static_tf(
                "left_camera/camera_link", "left_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            static_tf(
                "base_link", "right_camera/camera_link",
                (1.15, -0.65, 1.2 + wheel_radius), (0.0, 10.0, 290.0),
            ),
            static_tf(
                "right_camera/camera_link", "right_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
            # static_tf(
            #     "base_link", "rear_camera/camera_link",
            #     (-0.7, 0.0, 1.2 + wheel_radius), (0.0, 2.0, 180.0),
            # ),
            # static_tf(
            #     "rear_camera/camera_link", "rear_camera/camera_optical_link",
            #     (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            # ),
            # static_tf(
            #     "base_link", "rear_left_camera/camera_link",
            #     (0.05, 0.65, 1.2 + wheel_radius), (0.0, 10.0, 110.0),
            # ),
            # static_tf(
            #     "rear_left_camera/camera_link", "rear_left_camera/camera_optical_link",
            #     (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            # ),
            # static_tf(
            #     "base_link", "rear_right_camera/camera_link",
            #     (0.05, -0.65, 1.2 + wheel_radius), (0.0, 10.0, 250.0),
            # ),
            # static_tf(
            #     "rear_right_camera/camera_link", "rear_right_camera/camera_optical_link",
            #     (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            # ),
            static_tf(
                "base_link", "traffic_light_left_camera/camera_link",
                (-0.3, 0.0, 1.4 + wheel_radius), (0.0, 340.0, 0.0),
            ),
            static_tf(
                "traffic_light_left_camera/camera_link",
                "traffic_light_left_camera/camera_optical_link",
                (0.0, 0.0, 0.0), (-90.0, 0.0, -90.0),
            ),
        ]
    )
