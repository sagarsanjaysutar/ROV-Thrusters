from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node', 
            name='camera',
            output='screen',
             parameters=[{
                'video_device': '/dev/video0',
             }]
        ),
        Node(
            package='rov_project',
            executable='web_cam_publisher', 
            name='web_cam_publisher',
            output='screen'
        )
    ])