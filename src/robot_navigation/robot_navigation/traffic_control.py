import rclpy
from rclpy.node import Node
from rclpy.qos_event import QoSRequestedIncompatibleQoSInfo
from std_msgs.msg import String
from custom_interfaces.msg import TrafficControlRequest
import queue
import time


class TrafficControl(Node):
    def __init__(self):
        super().__init__('traffic_control_center')
        self.sub = self.create_subscription(TrafficControlRequest, 
                                            'unload_zone_traffic_control',
                                            self.receiveRequestCallback,
                                            10)
        self.pubisher = self.create_publisher(String, '/unload_zone_permission', 10)

        self.busy = False
        self.q = queue.Queue()
        self.completeCount = 0
        self.waitingCount = 0

    def receiveRequestCallback(self, msg):
        clientName = msg.name
        action = msg.action

        if action == "REQ_IN":
            self.get_logger().info(f'Receive request {action} from: {clientName}')
            self.enqueue(clientName)
            if not self.busy:
                time.sleep(5)
                self.sendNextPermission()

        elif action == "REQ_OUT":
            self.get_logger().info(f'Receive request {action} from: {clientName}')
            self.completeCount += 1
            self.get_logger().info(f'Complete count: {self.completeCount}')
            self.sendNextPermission()

        return

    def enqueue(self, clientName: str):
        self.waitingCount += 1
        self.get_logger().info(f'Enqueue {clientName}, current waiting count: {self.waitingCount}')
        self.q.put(clientName)

    def dequeue(self) -> str:
        self.waitingCount -= 1
        clientName = self.q.get()
        self.get_logger().info(f'Dequeue {clientName}, current waiting count: {self.waitingCount}')
        return clientName

    def sendNextPermission(self):
        if not self.q.empty():
            self.busy = True
            clientName = self.dequeue()
            self.get_logger().info(f'Send permission to topic /unload_zone_permission')
            
            msg = String()
            msg.data = clientName
            self.pubisher.publish(msg)
        else:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)

    trafficeControlCenter = TrafficControl()

    rclpy.spin(trafficeControlCenter)

    rclpy.shutdown()


if __name__ == '__main__':
    main()