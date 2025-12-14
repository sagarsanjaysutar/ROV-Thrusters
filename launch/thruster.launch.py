from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node', 
            name='thruster',
            output='screen',
             parameters=[{
                'fcu_url': 'udp://:14550@127.0.0.1:14555',
                'target_system_id' : 1,
                'target_component_id': 1
             }]
        ),
        Node(
            package='rov_project',
            executable='thruster_control', 
            name='thruster_control',
            output='screen'
        )
    ])