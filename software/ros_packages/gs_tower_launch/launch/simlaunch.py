import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    print(get_package_share_directory('sim_odrive'))
    print(get_package_share_directory('gs_tower_control'))

    sim_odrive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('sim_odrive'), 'launch'), '/launch.py']
        )
    )

    antenna_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gs_tower_control'), 'launch'), '/launch.py']
        )
    )

    return LaunchDescription([
        sim_odrive,
        antenna_control
    ])