#!/usr/bin/env python3


# Team ID:          [ Team-ID: 3166 ]
# Author List:		[Team Members: Labeeb, Amar, Abhinand, Dheeraj]
# Filename:		    task1b.py
# Functions:        [custom functions used: call_servo ,home_pose, go_to_pose, servo_move_1, servo_move_2, servo_move_3, servo_move_4]
# Nodes:		    node --> name of node created: servo_goal		 

from threading import Thread
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# my imports
from pymoveit2 import MoveIt2Servo
import numpy as np
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from linkattacher_msgs.srv import AttachLink, DetachLink


# variable tags for twist_stamped msgs to be used for servoing control
__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()

def main():
    '''
    This node is uses both movit2 and servoing method in combinations to implement a simulation pick-and-place actions.
    Both methods are run on the same node in sequence of actions.
    This method uses the servo to increase the accuracy of each coordinate the end effector reaches,
    hence, there is breakdown in the x-y-z velocity cmds.
    --> alternatively we can put into to single velocity vector to get one smooth motion to reach the required end-effector points,
        but the accuracy reduces  
    '''

    # global variables declared use to assign to each create_timer functions and clients
    global timer_1
    global timer_2
    global timer_3
    global timer_4
    global servo_client
    global gripper_attach_client
    global gripper_dettach_client
    
    rclpy.init()
    node = Node("servo_goal")
    
    # tf_ros.buffer is used for looking up transform between base_link and tool0 (located at the end-effector)
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    # clients for calling the attaching and dettaching services to attach or dettach the boxes from the EEF
    
    # variables and flags of importance of this node action
    drop = False
    # flags used the home_pose() and go_to_pose() custom fucntions (given below)
    home_flag_1 = 1
    home_flag_2 = 0
    joint_flag_1 = 1
    # set of flags to indicate the completion of individual subtasks that I divided the manipulator motions into
    is_task1_completed = False
    tolerance = 0.005 # controlling the error level in the manipluator end effector positions
    ssf = 1/5.0 # speed scaling factor



    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id= __twist_msg.header.frame_id,
        callback_group=callback_group,
    )

    def call_servo():
        '''
        Description: calls the servo trigger service 

        Args:
            
        Returns: 

        '''
        request = Trigger.Request()
        node.get_logger().info("Servo_node service started\n")
        future = servo_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        return future.result()

    def send_attach_request(model1_name):
        '''
        Description: call the attach link service and sends request based on which box to attach to the gripper of the manipulator 

        Args: mode1_name(string): string arguemnt that takes in the box name that should be attached to the gripper
              link1_name(string): "link"
            
        Returns: response(bool): returns either true or false indicating if the attachment is a success or not  

        '''

        request = AttachLink.Request()

        request.model1_name = model1_name
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"
        node.get_logger().info("Attach request sent\n")

        future = gripper_attach_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if(future.result() is not None):
            response = future.result()
            if(response.success == True):
                node.get_logger().info("Attachment done")
            else:
                node.get_logger().info("Attachment was not successful")
        else:
            node.get_logger().error("Failed to get response from the server")
        
    
    def send_detach_request(model1_name):
        '''
        Description: call the dettach link service and sends request to dettach the box from the gripper of the manipulator 

        Args: mode1_name(string): string arguemnt that takes in the box name that should be dettached
              link1_name(string): "link"
            
        Returns: response(bool): returns either true or false indicating if the dettachment is a success or not  

        '''

        request = DetachLink.Request()

        request.model1_name = model1_name
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"
        node.get_logger().info("Dettach request sent\n")

        future = gripper_detach_client.call_async(request)

        return future.result()
    
    def home_pose(home):
        '''
        Description:    Takes the manipulator back to the home position or alternate home position (initial pose of the maniuplator joints)

        Args:
            
            home(boolean): A flag argument that checks if the manipulator needs to go to the default home pose or alternate home pose(for pick-point-2)

        Returns:

        '''
        # the home positons of the manipulator joints 
        home_joint_position_1 = [0.0 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502] # got from intial.yaml file
        
        # got from simple observation of the e-yantraa instruction videos ( I call it the alternative home joint configuration)
        home_joint_position_2 = [-np.pi/2 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502] 

        if(home == 1):
            moveit2.move_to_configuration(joint_positions=home_joint_position_1)
            moveit2.wait_until_executed()
        elif(home == 0):
            moveit2.move_to_configuration(joint_positions=home_joint_position_2)
            moveit2.wait_until_executed()
        

    def go_to_pose(joint_config):
        '''
        Description:    Takes the manipulator to intermediate joint position near to drop off point

        Args:
            
            joint_config(boolean): A flag argument that to check which configuration the manipulator should take to updated its servoing workspace

        Returns:

        '''

        # global timer_1
        joint_position_1 = [0.0, -1.57, -1.57, -3.14, -1.57, 3.14]
        # joint_position_1 = [3.14 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502]
        joint_position_2 = [0.0 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502]
        
        if(joint_config == 1):
            moveit2.move_to_configuration(joint_positions=joint_position_1)
            moveit2.wait_until_executed()
        elif(joint_config == 0):
            moveit2.move_to_configuration(joint_positions=joint_position_2)
            moveit2.wait_until_executed()
        
    def pick_motion_1():
        '''
        Description:    Goes to the first pick up point and orients the manipulator to the drop-off point joint

        Args:

        Returns:

        '''
        global timer_1
        current_time = (node.get_clock().now().nanoseconds)*1e-9
        nonlocal is_task1_completed
        
        try:
            tr = tf_buffer.lookup_transform('base_link', 'obj_3', rclpy.time.Time())
        except tf2_ros.TransformException as e:
            node.get_logger().info(f'Could not transform base_link to obj_3: {e}')
            return
        try:
            tl = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
        except tf2_ros.TransformException as e:
            node.get_logger().info(f'Could not transform base_link to tool0: {e}')
            return
        
        # desired pose of box wtr to the base_link of the manipulator
        x_des = tr.transform.translation.x
        y_des = tr.transform.translation.y
        z_des = tr.transform.translation.z        
        # looked up pose of the EEF wrt base_link of manipulator
        x_pose = tl.transform.translation.x
        y_pose = tl.transform.translation.y
        z_pose = tl.transform.translation.z

        # coordinate errors
        err_x = np.abs(x_des - x_pose)
        err_y = np.abs(y_des - y_pose)
        err_z = np.abs(z_des - z_pose)

        print("x: %f, y: %f, z: %f\n" %(x_des, y_des, z_des))
        print("err_x: %f, err_y: %f, err_z: %f\n" %(err_x, err_y, err_z))

        # task one --> moving to pick-up point-1
        if(is_task1_completed == False):
            # first moving to pre-pick position
            if(err_y > tolerance):
                moveit2_servo(linear=(0.0, y_des*ssf, 0.0), angular=(0.0, 0.0, 0.0))
            elif(err_x > tolerance):
                moveit2_servo(linear=(x_des*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0)) 
            # moving to pickup point-1
            elif(err_z > tolerance):
                moveit2_servo(linear=(0.0, 0.0, z_des*ssf), angular=(0.0, 0.0, 0.0))
            # elif(err_y > tolerance):
            #     moveit2_servo(linear=(0.0, -y_des*ssf, 0.0), angular=(0.0, 0.0, 0.0))
            elif(err_y <= tolerance and err_x <= tolerance and err_z <= tolerance):
                moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                is_task1_completed = True
        # if task-1 completed then go to home joint config and then go to drop off joint config
        elif(is_task1_completed == True):
            gripper_attach_response = send_attach_request("box3")
            print(gripper_attach_response)
     

    # def attach():
    #     '''
    #     Description:   Orients the end effector to the correct drop off point and then orients itself to the alternative home joint configuration 

    #     Args:

    #     Returns:

    #     ''' 
    #     global timer_2
    #     global is_attached
    #     nonlocal is_task2_completed

    #     if(timer_1.is_canceled()):
    #         is_attached = send_attach_request("box3")
    #         timer_2.cancel()

    # def drop_motion():
    #     '''
    #     Description:   Goes to the second pick up point and orients the manipulator to the alternative home joint configuration 

    #     Args:

    #     Returns:

    #     ''' 
    #     global timer_3
    #     nonlocal is_task2_completed
    #     nonlocal is_home
    #     # Do this task after timer_1 and timer_2 is cancelled
    #     if(timer_1.is_canceled() and timer_2.is_canceled()):
    #         try:
    #             tr = tf_buffer.lookup_transform('base_link', 'obj_3', rclpy.time.Time())
    #         except tf2_ros.TransformException as e:
    #             node.get_logger().info(f'Could not transform base_link to obj_3: {e}')
    #             return
    #         try:
    #             tl = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
    #         except tf2_ros.TransformException as e:
    #             node.get_logger().info(f'Could not transform base_link to tool0: {e}')
    #             return
            
    #         # looked up pose of the EEF wrt base_link of manipulator
    #         x_pose = tl.transform.translation.x
    #         y_pose = tl.transform.translation.y
    #         z_pose = tl.transform.translation.z

    #         # coordinate errors
    #         err_x = np.abs(0.17 - x_pose)
    #         err_y = np.abs(-0.43 - y_pose)
    #         err_z = np.abs(0.17 - z_pose)

    #         if(is_task2_completed == False):
    #             if(err_x > tolerance):
    #                 moveit2_servo(linear=(-0.17, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
    #             elif(err_x <= tolerance):
    #                 moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
    #                 is_task2_completed = True
    #         elif(is_task2_completed == True):
    #             if(is_home == False):
    #                 home_pose(home_flag_1)
    #                 is_home = True
    #             elif(is_home == True):
    #                 go_to_pose(joint_flag_1)
    #                 timer_3.cancel()

    # # def detach():
    # #     '''
    # #     Description:   Orients the end effector to the correct drop off point and then orients itself to the alternative home joint configuration 

    # #     Args:

    # #     Returns:

    # #     ''' 
    # #     global timer_4
    # #     global is_detached

    # #     if(timer_1.is_canceled() and timer_2.is_canceled() and timer_3.is_canceled()):
    # #         is_detached = send_detach_request("box3")
    # #         timer_4.cancel()


    # def pick_motion_2():
    #     '''
    #     Description:    Orients the end effector to the correct drop off point and then orients itself to the default home joint configuration 

    #     Args:

    #     Returns:

    #     ''' 
    #     global timer_5
    #     nonlocal is_task3_completed 
    #     # Do this task after timer_1 and timer_2 and timer_3 is cancelled
    #     if(timer_1.is_canceled() and timer_2.is_canceled() and timer_3.is_canceled()):
    #         try:
    #             tr = tf_buffer.lookup_transform('base_link', 'obj_49', rclpy.time.Time())
    #         except tf2_ros.TransformException as e:
    #             node.get_logger().info(f'Could not transform base_link to obj_49: {e}')
    #             return
    #         try:
    #             tl = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
    #         except tf2_ros.TransformException as e:
    #             node.get_logger().info(f'Could not transform base_link to tool0: {e}')
    #             return
            
    #         # desired pose of box wtr to the base_link of the manipulator
    #         x_des = tr.transform.translation.x
    #         y_des = tr.transform.translation.y
    #         z_des = tr.transform.translation.z        
    #         # looked up pose of the EEF wrt base_link of manipulator
    #         x_pose = tl.transform.translation.x
    #         y_pose = tl.transform.translation.y
    #         z_pose = tl.transform.translation.z

    #         # coordinate errors
    #         err_x = np.abs(x_des - x_pose)
    #         err_y = np.abs(y_des - y_pose)
    #         err_z = np.abs(z_des - z_pose)

    #         home_pose(home_flag_1)


            
    # servo_node/servo_start client 
    servo_client = node.create_client(Trigger, "/servo_node/start_servo")
    # waiting for service
    while not servo_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('service not available, waiting again...')
    # function to call the servo service 
    servo_response = call_servo()

    # gripper attach and detach client
    gripper_attach_client = node.create_client(AttachLink, "GripperMagnetON")
    while not gripper_attach_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('EEF service not available, waiting again...')

    gripper_detach_client = node.create_client(DetachLink, "GripperMagnetOFF")
    while not gripper_detach_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('EEF service not available, waiting again...')



    # timer functions created to do tasks in a sequential manner and to avoid intermediate errors that came up when I ran movit2_joint pose immediately after servoing or vice versa
    timer_1 = node.create_timer(0.02, pick_motion_1)
    # timer_2 = node.create_timer(0.02, attach)
    # timer_3 = node.create_timer(0.02, drop_motion)
    # timer_4 = node.create_timer(0.02, detach)
    # timer_5 = node.create_timer(0.02, pick_motion_2)

    executor_2 = rclpy.executors.MultiThreadedExecutor(2) 
    executor_2.add_node(node)
    executor_2.spin()
    
    rclpy.shutdown()

    exit(0)



if __name__ == '__main__':
    main()