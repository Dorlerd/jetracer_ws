from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node'
        ),
        Node(
            package='jetson',
            executable='x_sensv2',
            name='x_sensv2',
            remappings=[('/velocity', '/velocity_IMU'),]
        )
    ])
