from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_dir  = get_package_share_directory('robot_navigation')
    paramFile = os.path.join(nav_dir, 'config', 'agv1_nav2_params.yaml')
    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='agv_demo',
            output='screen',
            parameters= [paramFile]
        )
    ]
    )