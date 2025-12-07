from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='voice_agent',
            executable='voice_to_action',
            name='voice_agent',
            output='screen'
        )
    ])
