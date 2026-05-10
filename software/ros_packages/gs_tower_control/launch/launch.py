from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'gs_tower_control',
            namespace = 'gs_tower_control',
            executable = 'antenna_control',
            name = 'antenna_control_node'
        )
    ])