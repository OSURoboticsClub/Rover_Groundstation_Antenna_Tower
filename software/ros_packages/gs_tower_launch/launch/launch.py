import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():

    odrive = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('odrive_can'), 'launch'), '/launch.yaml']
        )
    )

    antenna_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gs_tower_control'), 'launch'), '/launch.py']
        )
    )

    return LaunchDescription([
        odrive,
        antenna_control
    ])

