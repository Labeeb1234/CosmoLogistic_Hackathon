#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np


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
    '''
    
    Basic Navigator Node to move mobile robot autonomously from point A to B (using goToPoses())

    '''

    w0, x0, y0, z0 = euler_to_quaternion(0.0, 0.0, 0.0)

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

    # [x_pose, y_pose, yaw(euler)]  # tolerance level = pose --> +-0.3m and orient --> +-10 degrees or 0.174533
    p1 = [1.8, 1.5, 1.57]
    p2 = [2.0, -7.0, -1.57]
    p3 = [-3.0, 2.5, 1.57]
    goal_poses = [p1, p2, p3]
    goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    for i in range(0, len(goal_poses)):
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_poses[i][0]
        goal_pose.pose.position.y = goal_pose[i][1]
        goal_pose.pose.position.z = 0.0
        w, x, y, z = euler_to_quaternion(0.0, 0.0, goal_pose[i][2])
        goal_pose.pose.orientation.w = w
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.y = y
        goal_pose.pose.orientation.x = x

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        # move to the ith positon in the goal_poses array
        navigator.goToPose(goal_pose)
        j = 0
        while not navigator.isTaskComplete():
            j+=1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
            
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()
        
        result = navigator.getResult()
        if(result == TaskResult.SUCCEEDED):
            print("Goal pose Reached\n")
            i = i + 1
        elif(result == TaskResult.CANCELED):
            print("Goal pose Cancelled\n")
        elif(result == TaskResult.FAILED):
            print("Goal pose task failed\n")
        else:
            print("Invalid goal parameters passed\n")



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

    # i = 0
    # while not navigator.isTaskComplete():
    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')
        

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()


    # result_1 = navigator.getResult()
    # if result_1 == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result_1 == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result_1 == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    # if result_1 == TaskResult.SUCCEEDED:
    #     new_goal_pose = PoseStamped()
    #     new_goal_pose.header.frame_id = 'map'
    #     new_goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    #     new_goal_pose.pose.position.x = 2.0
    #     new_goal_pose.pose.position.y = -7.0
    #     new_goal_pose.pose.orientation.z = z2
    #     new_goal_pose.pose.orientation.w = w2
    #     new_goal_pose.pose.orientation.x = x2
    #     new_goal_pose.pose.orientation.y = y2
    #     navigator.goToPose(new_goal_pose)
        
    #     j = 0
    #     while not navigator.isTaskComplete():
    #         # Do something with the feedback
    #         j = j + 1
    #         feedback = navigator.getFeedback()
    #         if feedback and j % 5 == 0:
    #             print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #                 + ' seconds.')
            

    #             # Some navigation timeout to demo cancellation
    #             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #                 navigator.cancelTask()


    #     # [2.0, -7.0, -1.57]
    #     result_2 = navigator.getResult()
    #     if result_2 == TaskResult.SUCCEEDED:
    #         print('2nd Goal succeeded!')
    #     elif result_2 == TaskResult.CANCELED:
    #         print('Goal was canceled!')
    #     elif result_2 == TaskResult.FAILED:
    #         print('Goal failed!')
    #     else:
    #         print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()