from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
            package='rov_project',
            executable='thruster_control', 
            name='thruster_control',
            output='screen'
        )
    ])