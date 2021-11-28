from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes
from std_msgs.msg import String
from custom_interfaces.msg import TrafficControlRequest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class RobotController(Node):
    def __init__(self):
        super().__init__('agv')
        self.declare_parameters(
            namespace='',
            parameters=[
            ('initial_pose.x', None),
            ('initial_pose.y', None),
            ('initial_pose.rz', None),
            ('initial_pose.rw', None),
            ('wait_in_pose.x', None),
            ('wait_in_pose.y', None),
            ('wait_in_pose.rz', None),
            ('wait_in_pose.rw', None),
            ('unload_pose.x', None),
            ('unload_pose.y', None),
            ('unload_pose.rz', None),
            ('unload_pose.rw', None),
            ('wait_out_pose.x', None),
            ('wait_out_pose.y', None),
            ('wait_out_pose.rz', None),
            ('wait_out_pose.rw', None),
        ])

        self._initPoses()

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.permission = False

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        'compute_path_to_pose')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.requst_enter_pub = self.create_publisher(TrafficControlRequest, 'unload_zone_traffic_control', 10)
        self.permission_sub = self.create_subscription(String,
                                                       '/unload_zone_permission',
                                                       self._receivePermissionCallback,
                                                       10)



    def _initPoses(self):
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'        
        self.initial_pose.pose.position.x = self._getPararmeter('initial_pose.x')
        self.initial_pose.pose.position.y = self._getPararmeter('initial_pose.y')
        self.initial_pose.pose.orientation.z = self._getPararmeter('initial_pose.rz')
        self.initial_pose.pose.orientation.w = self._getPararmeter('initial_pose.rw')

        self.wait_in_pose = PoseStamped()
        self.wait_in_pose.header.frame_id = 'map'
        self.wait_in_pose.pose.position.x = self._getPararmeter('wait_in_pose.x')
        self.wait_in_pose.pose.position.y = self._getPararmeter('wait_in_pose.y')
        self.wait_in_pose.pose.orientation.z = self._getPararmeter('wait_in_pose.rz')
        self.wait_in_pose.pose.orientation.w = self._getPararmeter('wait_in_pose.rw')

        self.unload_pose = PoseStamped()
        self.unload_pose.header.frame_id = 'map'
        self.unload_pose.pose.position.x = self._getPararmeter('unload_pose.x')
        self.unload_pose.pose.position.y = self._getPararmeter('unload_pose.y')
        self.unload_pose.pose.orientation.z = self._getPararmeter('unload_pose.rz')
        self.unload_pose.pose.orientation.w = self._getPararmeter('unload_pose.rw')

        self.wait_out_pose = PoseStamped()
        self.wait_out_pose.header.frame_id = 'map'
        self.wait_out_pose.pose.position.x = self._getPararmeter('wait_out_pose.x')
        self.wait_out_pose.pose.position.y = self._getPararmeter('wait_out_pose.y')
        self.wait_out_pose.pose.orientation.z = self._getPararmeter('wait_out_pose.rz')
        self.wait_out_pose.pose.orientation.w = self._getPararmeter('wait_out_pose.rw')

    def goInitialPose(self):
        self.goToPose(self.initial_pose)

    def goWaitInPose(self):
        self.goToPose(self.wait_in_pose)

    def goUnloadPose(self):
        self.goToPose(self.unload_pose)

    def goWaitOutPose(self):
        self.goToPose(self.wait_out_pose)

    def notifyTrafficControlComplete(self):
        msg = TrafficControlRequest()
        msg.name = self.get_name()
        msg.action = 'REQ_OUT'
        self.requst_enter_pub.publish(msg)
    
    def requestInPermission(self):  
        self.permission = False
        msg = TrafficControlRequest()
        msg.name = self.get_name()
        msg.action = 'REQ_IN'
        self.requst_enter_pub.publish(msg)

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goToPose(self, pose):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        time.sleep(1)
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f'Following {len(goal_msg.poses)} goals....')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Cancel pending navigation request of any type."""
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        """Check if the navigation request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Goal with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitPermision(self):
        while not self.permission:
            rclpy.spin_once(self, timeout_sec=1.0)
            self.info('Wait permission again')
        return

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return
    
    def waitTrafficControlCenterActive(self):
        self._waitForNodeToActivate('traffic_control_center')
        return

    def getPath(self, start, goal):
        """Send a `ComputePathToPose` action request."""
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        if(self.feedback):
            self.debug('Remaining distance: ' + '{0:.0f}'.format(self.feedback.distance_remaining) + ' m.')
        return

    def _receivePermissionCallback(self, msg):
        self.info(f'Received permission msg: {msg.data}')
        if(msg.data == self.get_name()):
            self.info('Permission received')
            self.permission = True

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def _getPararmeter(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


class NavigationResult(Enum):
    UKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3