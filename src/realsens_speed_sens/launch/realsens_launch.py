from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():


    realsens = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py'])
            )
    listner = Node(
        package='test_jetson',
        executable='odom_cam',
        name='odom_cam',
        remappings=[('/velocity', '/velocit_realsens'),]
    )

    return LaunchDescription([realsens,listner])
