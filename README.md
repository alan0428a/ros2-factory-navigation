# ros2-factory-navigation
The application uses ros2 and nav2 to simulate a common scene in a factory. 

### SLAM

First we need to create the map using and turtlebot and cartographer node. The cartographer node will receive `/scan` topic from turtlebot sensor and then creat the map.
We need to launch the following nodes by order:

1. `ros2 launch factory_env factory.launch.py` Launch gazebo with pre-created world file.
2. `ros2 run robot_spawner spawner -n robot -x -3.0 -y 2.5 -z 0.01` Spawn a turtlebot at the factory world
3. `ros2 launch robot_spawner state_publisher.launch.py` Launch robot_state_publisher to publish the transforms corresponding to the movable joints of the robot.
4. `ros2 launch robot_slam slam.launch.py` Launch cartographer nodes to create map

5. `ros2 run turtlebot3_teleop teleop_keyboard` Lauch turtlebot keyboarder control interface

6. `ros2 run nav2_map_server map_saver_cli -f factory` Call save map command

