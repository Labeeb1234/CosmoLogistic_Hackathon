#!/usr/bin/env python3

# Team ID:          [ Team-ID: 3166 ]
# Author List:		[Team Members: Labeeb, Amar, Abhinand, Dheeraj]
# Filename:		    nav2_cmd_task1c.py
# Functions:        [custom functions used: euler_to_quaternion , quaternion_to_euler]
# Nodes:		    node --> name of node created: navigator		 


from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np
from rclpy.node import Node
import tf2_ros

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

def quaternion_to_euler(w, z, x, y):
    '''
    Description:    Convert quaternion angles (roll, pitch and yaw) to euler angles

    Args:
        z (float): stores the quaternion z component  
        x (float): stores the quaternion x component 
        y (float): stores the quaternion y component
        w (float) stores the quaternion w component
    Returns:
        A tuple representation of quaternion (roll, pitch, yaw)
    '''
    
    # roll angle calulations
    sinr_cosp = 2.0*(w*x + y*z)
    cosr_cosp = 1.0 - 2.0*(x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch angle caculations
    sinp = 2.0*(w*y - z*x)
    if(abs(sinp) >= 1.0):
        pitch = np.copysign(np.pi/2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # yaw angle calculations
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    rclpy.init()

    # Basic Navigator node constructor
    navigator = BasicNavigator()
    '''
    
    Basic Navigator Node to move mobile robot autonomously from point A to B (using goToPoses())

    '''
    # transform listener called
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)

    w0, x0, y0, z0 = euler_to_quaternion(0.0, 0.0, 0.0)

    # Setting the initial pose of the ebot
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


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # [x_pose, y_pose, yaw(euler)]  # tolerance level = pose --> +-0.3m and orient --> +-10 degrees or 0.174533
    p1 = [1.8, 1.5, 1.57]
    p2 = [2.0, -7.0, -1.57]
    p3 = [-3.0, 2.5, 1.57]
    goal_poses = [p1, p2, p3]  # array of goal poses
    goal_pose = PoseStamped()
    
    for i in range(0, len(goal_poses)):
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_poses[i][0]
        goal_pose.pose.position.y = goal_poses[i][1]
        goal_pose.pose.position.z = 0.0
        w, x, y, z = euler_to_quaternion(0.0, 0.0, goal_poses[i][2])
        goal_pose.pose.orientation.w = w
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.y = y
        goal_pose.pose.orientation.x = x

        # move to the ith positon in the goal_poses array
        navigator.goToPose(goal_pose)
        print(i)

        j = 0
        while not navigator.isTaskComplete():
            j+=1
            feedback = navigator.getFeedback()
            if feedback and j % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
            
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()
        
        result = navigator.getResult()
        if(result == TaskResult.SUCCEEDED):
            try:
                tr = tf_buffer.lookup_transform("map", "ebot_base_link", rclpy.time.Time())
            except tf2_ros.TransformExepection as e:
                navigator.get_logger().info(f"transform_lookup between odom and base_link not available: {e}")
                return
            # 2D position of the ebot base link            
            x_pose = tr.transform.translation.x
            y_pose = tr.transform.translation.y
            
            # lookup transform of the angles of the ebot_base_link
            w_orient = tr.transform.rotation.w
            z_orient = tr.transform.rotation.z
            x_orient = tr.transform.rotation.x
            y_orient = tr.transform.rotation.y

            # conversion of quaternions to euler angles
            roll, pitch, yaw = quaternion_to_euler(w_orient, z_orient, x_orient, y_orient)

            # pose errors
            err_x = np.abs(x_pose-goal_poses[i][0])
            err_y = np.abs(y_pose-goal_poses[i][1])
            err_yaw = np.abs(yaw-goal_poses[i][2])
            
            # logged values of the pose
            navigator.get_logger().info(f"x: {x_pose}, y: {y_pose}, yaw(rad): {yaw}\n")
            navigator.get_logger().info(f"err_x: {err_x}, err_y: {err_y}, err_yaw(rad): {err_yaw}\n")
            
            print("Goal pose Reached\n")
            i += 1 # index update
        
        elif(result == TaskResult.CANCELED):
            print("Goal pose Cancelled\n")
        elif(result == TaskResult.FAILED):
            print("Goal pose task failed\n")
        else:
            print("Invalid goal parameters passed\n")

    navigator.destroy_node()

    exit(0)


if __name__ == '__main__':
    main()