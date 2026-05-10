from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package = 'sim_odrive',
            namespace = 'pan_axis',
            executable = 'sim_odrive',
            name = 'sim_odrive_pan',
            parameters = [
                {"maxVelocity": 500.0}
            ]
        ),
        Node (
            package = 'sim_odrive',
            namespace = 'elev_axis',
            executable = 'sim_odrive',
            name = 'sim_odrive_elev',
            parameters = [
                {"maxVelocity": 500.0}
            ]
        )
    ])