import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로
    package_dir = get_package_share_directory('udp_camera_receiver')

    # 파라미터 파일 경로
    param_file = os.path.join(package_dir, 'config', 'param.yaml')

    # 노드 생성
    udp_camera_receiver_node = Node(
        package='udp_camera_receiver',
        executable='udp_camera_receiver_node',
        name='udp_camera_receiver',
        output='screen',
        parameters=[param_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        udp_camera_receiver_node,
    ])
