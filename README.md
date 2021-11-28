# ros2-factory-navigation
The application uses ros2 and nav2 to simulate a common scene in a factory. 

## Mapping(SLAM)

First we need to create the map using and turtlebot and cartographer node. The cartographer node will receive `/scan` topic from turtlebot sensor and then creat the map.
We need to launch the following nodes by order:

1. `ros2 launch factory_env factory.launch.py` Launch gazebo with pre-created world file.
1. `ros2 run robot_spawner spawner -n robot -x -1.0 -y 2.5` Spawn a turtlebot at the factory world
1. `ros2 launch robot_spawner state_publisher.launch.py` Launch robot_state_publisher to publish the transforms corresponding to the movable joints of the robot.
1. `ros2 launch robot_slam slam.launch.py` Launch cartographer nodes to create map

1. `export TURTLEBOT3_MODEL=waffel_pi` Set robot model for keyboard control interface
1. `ros2 run turtlebot3_teleop teleop_keyboard` Lauch turtlebot keyboarder control interface

1. `ros2 run nav2_map_server map_saver_cli -f factory` Call save map command. This map will be used in the following Localization step.

## Localization

In the localization step, we will provide the map file we just created at Mapping stage to nav2_nav_server node. This map and sensor data will be used to calculated robot's position using amcl algorithm.

1. Run step 1~3 at Mapping stage
1. `ros2 launch robot_navigation amcl.launch.py` Launch nav2 related nodes and rviz.
1. `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}"` We call the `/reinitialize_global_localization` to reset probability(amcl particle cloud) at every position. 
1. `export TURTLEBOT3_MODEL=waffel_pi` Set robot model for keyboard control interface
1. `ros2 run turtlebot3_teleop teleop_keyboard` Lauch turtlebot keyboarder control interface
1. Observe the how the amcl particle cloud changes when moving the robot aroung. The amcl particle cloud will eventually gather around the robot, which means that the localization node believes its calculated position is around the robot.

## Navigation

1. `ros2 launch factory_env factory.launch.py` Launch gazebo with pre-created world file.
1. `ros2 launch robot_navigation amcl.launch.py` Launch nav2 related nodes and rviz.
1. `ros2 run robot_spawner spawner -n robot -x -1.0 -y 2.5 -pub True` Spawn a turtlebot at the factory world and publish initial position
1. `ros2 launch robot_spawner state_publisher.launch.py` Launch robot_state_publisher to publish the transforms corresponding to the movable joints of the robot.
1. `ros2 launch robot_navigation navigation.launch.py` Launch nav2 navigation stack
1. Use the "Navigation2 Goal" button on Rviz interface to set goal pose. The nav2 stacks will plan a shortest path using local and global costmap shown on Rviz.


`ros2 run robot_spawner spawner -n robot -x -2.0 -y 2.5` 