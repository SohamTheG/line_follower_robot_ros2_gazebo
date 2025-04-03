from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower_pid',
            executable='line_follower',
            name='line_follower_pid',
            output='screen'
        )
    ])