import os
import sys
import rclpy
import argparse
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped
import xml.etree.ElementTree as ET


class Spawner(Node):
    def __init__(self, args):
        super().__init__('spawner')
        self.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)

        self.args = args

    def send_request(self):
        # Get path to the turtlebot3 burgerbot
        sdf_file_path = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"), "models",
            "turtlebot3_waffle", "model.sdf")

        tree = ET.parse(sdf_file_path)
        root = tree.getroot()

        self.get_logger().info(f'Namespace is {self.args.namespace}')
        if(self.args.namespace != ''):
            # We need to remap the transform (/tf) topic so each robot has its own.
            # We do this by adding `ROS argument entries` to the sdf file for
            # each plugin broadcasting a transform. These argument entries provide the
            # remapping rule, i.e. /tf -> /<robot_id>/tf
            for plugin in root.iter('plugin'):
                # TODO(orduno) Handle case if an sdf file from non-turtlebot is provided
                if 'turtlebot3_diff_drive' in plugin.attrib.values():
                    # The only plugin we care for now is 'diff_drive' which is
                    # broadcasting a transform between`odom` and `base_footprint`
                    break

            ros_params = plugin.find('ros')
            ros_tf_remap = ET.SubElement(ros_params, 'remapping')
            ros_tf_remap.text = '/tf:=/' + self.req.robot_namespace + '/tf'        

        self.req.name = self.args.name
        self.req.xml = ET.tostring(root, encoding='unicode')
        self.req.robot_namespace = self.args.namespace
        self.req.initial_pose.position.x = float(self.args.initialX)
        self.req.initial_pose.position.y = float(self.args.initialY)
        self.req.initial_pose.position.z = float(self.args.initialZ)

        if(self.args.publish_initial_position):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = float(self.args.initialX)
            msg.pose.pose.position.y = float(self.args.initialY)
            msg.pose.pose.position.z = float(self.args.initialZ)
            msg.pose.pose.orientation.w = float(1.0)
            self.get_logger().info(f'Publishing Initial Position  \n X= {float(self.args.initialX)} \n Y = {float(self.args.initialY)} \n W = {float(self.args.rotation_z)}')
            self.publisher.publish(msg)

        self.get_logger().info(
            f"Set robot name: {self.req.name}, namespace: {self.req.robot_namespace}, at x: {self.req.initial_pose.position.x} y: {self.req.initial_pose.position.y} z: {self.req.initial_pose.position.z}")
        self.get_logger().info("Sending service request to `/spawn_entity`")
        self.future = self.cli.call_async(self.req)

def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', default='robot')
    parser.add_argument('-ns','--namespace', default='')
    parser.add_argument('-x', '--initialX', default=0.0, type = float)
    parser.add_argument('-y', '--initialY', default=0.0, type=float)
    parser.add_argument('-z', '--initialZ', default=0.0, type=float)
    parser.add_argument('-w', '--rotation_z', default=0.0, type=float)
    parser.add_argument('-pub', '--publish_initial_position', default=False, type=bool)
    return parser

def main(args=None):
    rclpy.init(args=args)
    parser = get_parser()
    args, unknown = parser.parse_known_args()
    spawner_client = Spawner(args)
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
        
