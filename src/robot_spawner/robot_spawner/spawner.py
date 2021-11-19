import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

class Spawner(Node):
    def __init__(self, args):
        super().__init__('spawner')
        self.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.args = args

    def send_request(self):
        # Get path to the turtlebot3 burgerbot
        sdf_file_path = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"), "models",
            "turtlebot3_waffle", "model.sdf")

        self.req.name = self.args[0]
        self.req.xml = open(sdf_file_path, 'r').read()
        self.req.robot_namespace = self.args[1]
        self.req.initial_pose.position.x = float(self.args[2])
        self.req.initial_pose.position.y = float(self.args[3])
        self.req.initial_pose.position.z = float(self.args[4])

        self.get_logger().info(
            f"Set robot name: {self.req.name}, namespace: {self.req.robot_namespace}, at x: {self.req.initial_pose.position.x} y: {self.req.initial_pose.position.y} z: {self.req.initial_pose.position.z}")
        self.get_logger().info("Sending service request to `/spawn_entity`")
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    spawner_client = Spawner(sys.argv[1:])
    spawner_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(spawner_client)
        if spawner_client.future.done():
            try:
                response = spawner_client.future.result()
            except Exception as e:
                spawner_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                spawner_client.get_logger().info('response: %r' % spawner_client.future.result())
            break

    spawner_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
