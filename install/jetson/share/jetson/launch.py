from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():
    realsens = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("realsens_jetson"), '/realsens_launch.py']))
    xsense = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("jetson"), '/x_launch.py']))
    kalman = Node(
            package='kalman_filter_jetracer',
            executable='kalman',
            name='kalman',
        )


    return LaunchDescription([realsens,xsense,kalman])
