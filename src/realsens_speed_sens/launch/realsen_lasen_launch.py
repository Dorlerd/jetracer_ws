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
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='odom_cam',
        remappings=[('/depth', '/camera/depth/image_rect_raw'),('/depth_camera_info', '/camera/depth/camera_info'),]
    )

    return LaunchDescription([realsens,listner])
