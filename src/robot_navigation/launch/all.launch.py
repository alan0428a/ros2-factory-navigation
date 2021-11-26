from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    factory_env_dir = get_package_share_directory('factory_env')    
    robot_spawner_dir = get_package_share_directory('robot_spawner')

    gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(factory_env_dir, 'launch', 'factory.launch.py')))
    
    amcl_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(robot_navigation_dir, 'launch', 'amcl.launch.py')))
    
    # spawner_cmd = Node(
    #     package='robot_spawner',
    #     executable='spawner',
    #     arguments=['-n', 'robot', '-x', '-1.0', '-y', '2.5'],
    #     output='screen')
    
    publisher_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(robot_spawner_dir, 'launch', 'state_publisher.launch.py')))

    nav_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(robot_navigation_dir, 'launch', 'navigation.launch.py')))
    
    ld = LaunchDescription()

    ld.add_action(gazebo_cmd)
    ld.add_action(amcl_cmd)
    # ld.add_action(spawner_cmd)
    ld.add_action(publisher_cmd)
    ld.add_action(nav_cmd)

    return ld