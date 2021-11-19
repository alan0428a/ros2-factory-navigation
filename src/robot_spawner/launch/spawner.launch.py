from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_spawner',
            executable='spawner',
            output='screen'
        )
    ]
    )