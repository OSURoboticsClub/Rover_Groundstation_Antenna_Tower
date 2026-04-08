import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
   #Example of grabbing and using launch file
   #nav_autonomy = IncludeLaunchDescription(
   #   PythonLaunchDescriptionSource([os.path.join(
   #      get_package_share_directory('nav_autonomy'),
   #      'launch'), '/nav_launch.py'])
   #A   )
   return LaunchDescription([
     #put launch files here - nav_autonomy
   ])
