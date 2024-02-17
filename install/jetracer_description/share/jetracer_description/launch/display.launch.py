from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('jetracer_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'jetracer.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ])
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
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
    
    joint_state_broadcaster_spawner = Node(
        package ="controller_manager",
        executable='spawner',
        arguments=['joint_broad', '--controller-manager',
            '/controller_manager'],
    )
    
    joint_steering = Node(
        package ="controller_manager",
        executable='spawner',
        arguments=['joint_steering', '-c',
            '/controller_manager'],
    )
    
    
    joint_wheels = Node(
        package ="controller_manager",
        executable='spawner',
        arguments=['joint_wheels', '-c',
            '/controller_manager'],
    )


    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [joint_steering,joint_wheels],
            )
        ),
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        joint_state_broadcaster_spawner
    ])

