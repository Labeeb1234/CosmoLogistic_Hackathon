#!/usr/bin/env python3


from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np
from rclpy.node import Node
import tf2_ros
from tf_transformations import euler_from_quaternion
# custom service imports
from ebot_docking.srv import DockSw 
from linkattacher_msgs.srv import AttachLink, DetachLink


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

    # Basic Navigator node constructor
    navigator = BasicNavigator()
    '''
    
    --> Basic Navigator Node to move mobile robot autonomously from point A to B (using goToPoses())
    --> building upon the code of Task_1C, this node moves bot.....

    '''
    # required parameters being called and stores in variables here 
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, navigator)
    # creating the docking client
    dock_server_client = navigator.create_client(DockSw, "start_docking")
    link_attach_client = navigator.create_client(AttachLink, "ATTACH_LINK")
    dock_req = DockSw.Request()
    attach_req = AttachLink.Request()


    def send_dock_request(need_linear_dock, need_orientaion_dock, rack_angle, rack_id):
        dock_req.linear_dock = need_linear_dock
        dock_req.orientation_dock = need_orientaion_dock
        dock_req.orientation = rack_angle
        dock_req.rack_no = rack_id

        future = dock_server_client.call_async(dock_req)
        print("Sending docking request parameters\n")
        rclpy.spin_until_future_complete(navigator, future)
        
        return future.result()
    
    def send_rack_attach_request(model1_name, model2_name):
        attach_req.model1_name = model1_name
        attach_req.link1_name = "ebot_base_link"
        attach_req.model2_name = model2_name
        attach_req.link2_name = "link"
        
        future = link_attach_client.call_async(attach_req)
        print("Attaching rack to bot\n")
        rclpy.spin_until_future_complete(navigator,future)

        return future.result()


    # home pose angle 
    w0, x0, y0, z0 = euler_to_quaternion(0.0, 0.0, 0.0)

    # Setting the initial pose(home pose) of the ebot
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

    def go_home():
        navigator.goToPose(initial_pose)
        
        m = 0
        while not navigator.isTaskComplete():
            m+=1
            home_feedback = navigator.getFeedback()
            if(home_feedback and m%5 == 0):
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.\n')
                print("Going home\n")

        result = navigator.getResult()
        if(result == TaskResult.SUCCEEDED):
            print("Home pose Reached\n")
        elif(result == TaskResult.CANCELED):
            print("Pose Cancelled\n")
        elif(result == TaskResult.FAILED):
            print("Pose task failed\n")
        else:
            print("Invalid goal parameters passed\n")



    # [x_pose, y_pose, yaw(euler)]  # tolerance level = pose --> +-0.3m and orient --> +-10 degrees or 0.174533
    goal_pose = PoseStamped()
    
    rack_goal = [0.5, 4.35, 2.5] 
    

    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = rack_goal[0]
    goal_pose.pose.position.y = rack_goal[1]
    goal_pose.pose.position.z = 0.0
    w, x, y, z = euler_to_quaternion(0.0, 0.0, rack_goal[2])
    goal_pose.pose.orientation.w = w
    goal_pose.pose.orientation.z = z
    goal_pose.pose.orientation.y = y 
    goal_pose.pose.orientation.x = x

        # move to the ith positon in the goal_poses array
    navigator.goToPose(goal_pose)

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
        print("Goal pose Reached\n")
        dock_response = send_dock_request(need_linear_dock=True, need_orientaion_dock=True, rack_angle=3.14, rack_id='1')
        navigator.get_logger().info(f"{dock_response}\n")
        if(dock_response.success == True):
            # attach rack link on bot
            attach_link_response = send_rack_attach_request("ebot", "rack1")
            navigator.get_logger().info(f"{attach_link_response}\n")
            
        else:
            # don't attach link on bot
            pass
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