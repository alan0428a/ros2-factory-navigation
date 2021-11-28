import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    param_substitutions = {}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='agv_auto',
            output='screen',
            parameters= [configured_params]
        )
    ]
    )