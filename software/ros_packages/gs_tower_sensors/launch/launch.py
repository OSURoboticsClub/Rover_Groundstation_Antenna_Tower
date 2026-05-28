from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'gs_tower_sensors',
            namespace = 'gs_tower_sensors',
            executable = 'imu',
            name = 'imu',
            parameters = [
                {"magnetic_declination": 9.91}
            ]
        ),
        Node (
            package = 'gs_tower_sensors',
            namespace = 'gs_tower_sensors',
            executable = 'gps',
            name = 'gps'
        )
    ])