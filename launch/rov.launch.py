from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
            package='rov_project',
            executable='tele_cmd_publisher', 
            name='tele_cmd_publisher',
            output='screen'
        ),
        Node(
            package='rov_project',
            executable='thruster_publisher',
            name='thruster_mixer_node',
            output='screen'
        ),])