import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Speak Server Node
    sync_template_server_node = Node(
        package='speak_server',
        executable='speak_server_node',
        namespace='waypoint_function',
        output='screen',
    )
    ld.add_action(sync_template_server_node)

    # voicevox_ros2 Node
    voicevox_ros2_node = Node(
        package='voicevox_ros2',
        executable='voicevox_ros2',
        name='voicevox_ros2_node',
        output='screen',
    )
    ld.add_action(voicevox_ros2_node)

    return ld