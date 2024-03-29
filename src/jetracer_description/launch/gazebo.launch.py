from launch_ros.actions import Node
import launch
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('jetracer_description')
    world_path=os.path.join(share_dir, 'world/my_world.sdf')
    xacro_file = os.path.join(share_dir, 'urdf', 'jetracer.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )


    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jetracer',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_file, description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        urdf_spawn_node,
        rviz_node
    ])
