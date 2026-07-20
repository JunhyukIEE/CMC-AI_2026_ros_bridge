import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nodes = []

    # Ego vehicle converter
    ego_node = Node(
        package='localization_converter',
        executable='localization_converter_node',
        name='localization_converter_node',
        namespace='Ego',
        parameters=[{
            'ego_topic': '/Ego_topic',
            # 'gps_topic': '/gps',
            # 'imu_topic': '/imu',
            'odom_topic': 'odom',
            'twist_topic': 'twist',
            'pose_topic': 'pose',
            # 'gps_fix_topic': 'gps/fix',
            # 'imu_corrected_topic': 'imu/corrected',
        }],
        output='screen',
    )
    nodes.append(ego_node)

    # # Agent 1 converter
    # agent1_node = Node(
    #     package='localization_converter',
    #     executable='localization_converter_node',
    #     name='localization_converter_node',
    #     namespace='Agent1',
    #     parameters=[{
    #         'ego_topic': '/Agent1_topic',
    #         'gps_topic': '/gps',
    #         'imu_topic': '/imu',
    #         'odom_topic': 'odom',
    #         'twist_topic': 'twist',
    #         'pose_topic': 'pose',
    #         'gps_fix_topic': 'gps/fix',
    #         'imu_corrected_topic': 'imu/corrected',
    #     }],
    #     output='screen',
    # )
    # nodes.append(agent1_node)

    # # Agent 2 converter
    # agent2_node = Node(
    #     package='localization_converter',
    #     executable='localization_converter_node',
    #     name='localization_converter_node',
    #     namespace='Agent2',
    #     parameters=[{
    #         'ego_topic': '/Agent2_topic',
    #         'gps_topic': '/gps',
    #         'imu_topic': '/imu',
    #         'odom_topic': 'odom',
    #         'twist_topic': 'twist',
    #         'pose_topic': 'pose',
    #         'gps_fix_topic': 'gps/fix',
    #         'imu_corrected_topic': 'imu/corrected',
    #     }],
    #     output='screen',
    # )
    # nodes.append(agent2_node)

    # # Agent 3 converter
    # agent3_node = Node(
    #     package='localization_converter',
    #     executable='localization_converter_node',
    #     name='localization_converter_node',
    #     namespace='Agent3',
    #     parameters=[{
    #         'ego_topic': '/Agent3_topic',
    #         'gps_topic': '/gps',
    #         'imu_topic': '/imu',
    #         'odom_topic': 'odom',
    #         'twist_topic': 'twist',
    #         'pose_topic': 'pose',
    #         'gps_fix_topic': 'gps/fix',
    #         'imu_corrected_topic': 'imu/corrected',
    #     }],
    #     output='screen',
    # )
    # nodes.append(agent3_node)

    return LaunchDescription(nodes)
