#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np
"""

Basic navigation to go to a given poses


"""

def euler_to_quaternion(roll, pitch, yaw):
    '''
    Description:    Convert Euler angles (roll, pitch and yaw) to quaternion 

    Args:
        roll (float): Roll angle in radians 
        pitch (float): Pitch angle in radians
        yaw (float): Yaw angle in radians

    Returns:
        A tuple representation of quaternion (w, x, y, z)
    '''

    cy = np.cos(yaw*0.5)
    sy = np.sin(yaw*0.5)
    cp = np.cos(pitch*0.5)
    sp = np.sin(pitch*0.5)
    cr = np.cos(roll*0.5)
    sr = np.sin(roll*0.5)

    w = cy*cp*cr + sy*sp*sr
    x = cy*cp*sr - sy*sp*cr
    y = sy*cp*sr + cy*sp*cr
    z = sy*cp*cr - cr*sp*sr 
    return w, x, y, z


def main():
    rclpy.init()

    navigator = BasicNavigator()

    w0, x0, y0, z0 = euler_to_quaternion(0.0, 0.0, 0.0)
    w1, x1, y1, z1 = euler_to_quaternion(0.0, 0.0, np.pi/2+0.174533)
    w2, x2, y2, z2 = euler_to_quaternion(0.0, 0.0, -np.pi/2-0.174533)

    # Setting our initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = z0
    initial_pose.pose.orientation.w = w0
    initial_pose.pose.orientation.x = x0
    initial_pose.pose.orientation.y = y0

    navigator.setInitialPose(initial_pose)
    # [1.8, 1.5, 1.57]
    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_poses = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.8
    goal_pose.pose.position.y = 1.5
    goal_pose.pose.orientation.z = z1
    goal_pose.pose.orientation.w = w1
    goal_pose.pose.orientation.x = x1
    goal_pose.pose.orientation.y = y1
    goal_poses.append(goal_pose)
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    # new_goal_pose = PoseStamped()
    # new_goal_pose.header.frame_id = 'map'
    # new_goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # new_goal_pose.pose.position.x = 2.0
    # new_goal_pose.pose.position.y = -7.0
    # new_goal_pose.pose.orientation.z = z2
    # new_goal_pose.pose.orientation.w = w2
    # new_goal_pose.pose.orientation.x = 0.0
    # new_goal_pose.pose.orientation.y = 0.0
    # goal_poses.append(new_goal_pose)


    navigator.goToPose(goal_pose)
    # navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()


    result_1 = navigator.getResult()
    if result_1 == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result_1 == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result_1 == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    if result_1 == TaskResult.SUCCEEDED:
        new_goal_pose = PoseStamped()
        new_goal_pose.header.frame_id = 'map'
        new_goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        new_goal_pose.pose.position.x = 2.0
        new_goal_pose.pose.position.y = -7.0
        new_goal_pose.pose.orientation.z = z2
        new_goal_pose.pose.orientation.w = w2
        new_goal_pose.pose.orientation.x = x2
        new_goal_pose.pose.orientation.y = y2
        navigator.goToPose(new_goal_pose)
        
        j = 0
        while not navigator.isTaskComplete():
            # Do something with the feedback
            j = j + 1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
            

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()


        # [2.0, -7.0, -1.57]
        result_2 = navigator.getResult()
        if result_2 == TaskResult.SUCCEEDED:
            print('2nd Goal succeeded!')
        elif result_2 == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result_2 == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()