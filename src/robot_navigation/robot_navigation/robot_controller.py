from geometry_msgs.msg import PoseStamped
from .robot_navigator import BasicNavigator, NavigationResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.0
    initial_pose.pose.position.y = 2.5
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # set our demo's goal poses
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.0
    goal_pose1.pose.position.y = 1.5
    goal_pose1.pose.orientation.z = 0.0
    goal_pose1.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose1)

    # additional goals can be appended
    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = 'map'
    # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose2.pose.position.x = 4.0
    # goal_pose2.pose.position.y = 0.0
    # goal_pose2.pose.orientation.z = 0.0
    # goal_pose2.pose.orientation.w = 1.0
    # goal_poses.append(goal_pose2)
    # goal_pose3 = PoseStamped()
    # goal_pose3.header.frame_id = 'map'
    # goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose3.pose.position.x = -3.6
    # goal_pose3.pose.position.y = -4.75
    # goal_pose3.pose.orientation.w = 0.707
    # goal_pose3.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose3)

    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    # navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Remaining distance: ' + '{0:.0f}'.format(feedback.distance_remaining)
                  + ' m.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
            #     goal_pose4.pose.position.x = -5.0
            #     goal_pose4.pose.position.y = -4.75
            #     goal_pose4.pose.orientation.w = 0.707
            #     goal_pose4.pose.orientation.z = 0.707
            #     navigator.goThroughPoses([goal_pose4])

    # Do something depending on the return code
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()