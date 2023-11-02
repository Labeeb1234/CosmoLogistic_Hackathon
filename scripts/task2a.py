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
    global gripper_detach_client
    
    rclpy.init()
    node = Node("servo_goal")

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
    
    # tf_ros.buffer is used for looking up transform between base_link and tool0 (located at the end-effector)
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    # clients for calling the attaching and dettaching services to attach or dettach the boxes from the EEF
    
    # variables and flags of importance of this node action
    drop = False
    attached = False
    
    go_home = False
    is_home = False
    
    to_pick = True

    is_home_2 = False
    is_home_3 = False

    # flags used the home_pose() and go_to_pose() custom fucntions (given below)
    home_flag_1 = 1
    home_flag_2 = 0
    joint_flag_1 = 1
    joint_flag_2 = 2
    joint_flag_3 = 3
    
    # set of flags to indicate the completion of individual subtasks that I divided the manipulator motions into
    is_task1_completed = False
    is_task2_completed = False
    is_task3_completed = False

    tolerance = 0.005 # controlling the error level in the manipluator end effector positions\
    tolerance_2 = 0.05 # secondary safety tolerance level
    ssf = 1/5.0 # speed scaling factor

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
            
        Returns: (bool): returns True 

        '''
        request = AttachLink.Request()

        request.model1_name = model1_name
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"
        node.get_logger().info(f"sending link attachment request")

        gripper_attach_client.call_async(request)
        return True

    def send_detach_request(model1_name):
        '''
        Description: call the dettach link service and sends request to dettach the box from the gripper of the manipulator 

        Args: mode1_name(string): string arguemnt that takes in the box name that should be dettached
            
        Returns: response(bool): returns True   

        '''

        request = DetachLink.Request()

        request.model1_name = model1_name
        request.link1_name = "link"
        request.model2_name = "ur5"
        request.link2_name = "wrist_3_link"
        node.get_logger().info("Dettach request sent\n")

        gripper_detach_client.call_async(request)

        return True
    
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
        # joint_position_1 = [0.0, -1.57, -1.57, -3.14, -1.57, 3.14]
        # joint_position_1 = [0.0, -1.7576610836374074, -1.6402959918423714, -2.832839350379115, -1.5698630409337033, 3.13900700443266]
        joint_position_1 = [0.0, -1.6462654766267248, -1.7748648694170135, -2.8088456154373436, -1.568476497881539, 3.1389856266354634]
        # joint_position_2 = [0.0 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502]
        joint_position_2 = [0.0, -2.224606098948603, -0.7492140446316413, -3.2202816291841163, -1.5695779315060836, 3.1371470124197303]
        # joint_position_3 = []
        
        if(joint_config == 1):
            moveit2.move_to_configuration(joint_positions=joint_position_1)
            moveit2.wait_until_executed()
        elif(joint_config == 2):
            moveit2.move_to_configuration(joint_positions=joint_position_2)
            moveit2.wait_until_executed()

    def drop_motion(timer_, home_flag_, joint_flag_, box):
        '''
        Description: Goes to the drop position and drops the box

        Args: 
            
            timer_(timer function) - takes in the timer function variable 
            home_flag_(bool) - a boolean arguement that indicates whether the manipulator is in home position or not
            joint_flag_(bool) - a boolean arguement that indicates which joint position the manipulator has to take to drop the box
            box(string) - takes in a string value indicating the box that needs to be dropped

        Returns:

        ''' 
        nonlocal drop
        nonlocal is_home

        if(is_home == False):
            # pre-drop position
            home_pose(home_flag_)
            node.get_logger().info("Going home...")
            is_home = True
        elif(is_home == True):
            # moving to drop configuration
            if(drop == False):
                go_to_pose(joint_flag_)
                # timer_1.cancel()
                drop = True
            elif(drop == True):
                detached = send_detach_request(box)
                node.get_logger().info("Box placed..!")
                if(detached == True):
                    if(timer_ == timer_1):
                        # reset the required flags for the next drop
                        drop = False
                        is_home = False
                        timer_1.cancel()
                    elif(timer_ == timer_2):
                        # reset the required flags for the next drop
                        drop = False
                        is_home = False
                        timer_2.cancel()
                    elif(timer_ == timer_3):
                        drop = False
                        is_home = False
                        timer_3.cancel()
                else:
                    node.get_logger().info("something went wrong!")
                    if(timer_ == timer_1):
                        drop = False
                        is_home = False
                        timer_1.cancel()
                    elif(timer_ == timer_2):
                        drop = False
                        is_home = False
                        timer_2.cancel()

    # timer functions that gets a callback each 0.02 sec    
    def pick_motion_1():
        '''
        Description: Goes to the picks and then drops

        Args:

        Returns:

        '''
        # global timer_1

        nonlocal is_task1_completed
        nonlocal attached        
        nonlocal to_pick

        if(to_pick == True):
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

            # safety limits so that the rack does not move while manipulator goes to the drop config
            dis_safe = 0.15
            
            # coordinate errors
            err_x = np.abs(x_des - x_pose)
            err_y = np.abs(y_des - y_pose)
            err_z = np.abs(z_des - z_pose)

            err_rad = (x_des-x_pose)**2 + (y_des-y_pose)**2 + (z_des-z_pose)**2 

            node.get_logger().info(f"x: {x_des}, y: %{y_des}, z: {z_des}\n")
            node.get_logger().info(f"err_x: {err_x}, err_y: {err_y}, err_z: {err_z}, err_dis: {err_rad}\n")

            # task one --> moving to pick-up point-1
            if(is_task1_completed == False):
                # moving to pre-pick position (in two steps --> have to change to single step)
                if(err_y > tolerance):
                    moveit2_servo(linear=(0.0, y_des*ssf, 0.0), angular=(0.0, 0.0, 0.0))
                elif(err_x > tolerance):
                    moveit2_servo(linear=(x_des*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0)) 
                # moving to pickup point-1
                elif(err_z > tolerance):
                    moveit2_servo(linear=(0.0, 0.0, z_des*ssf), angular=(0.0, 0.0, 0.0))
                elif(err_y <= tolerance and err_x <= tolerance and err_z <= tolerance):
                    moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                    is_task1_completed = True
        # if task-1 completed then go to home joint config and then go to drop off joint config
            elif(is_task1_completed == True):
                attached = send_attach_request("box3")
                node.get_logger().info(f"{attached}")
                if(attached == True):
                    if(np.abs(dis_safe-x_pose) > tolerance_2):
                        moveit2_servo(linear=(-dis_safe*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                    elif(np.abs(dis_safe-x_pose) <= tolerance_2):
                        moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                        to_pick = False
        else:
            drop_motion(timer_=timer_1, home_flag_=home_flag_1, joint_flag_=joint_flag_1, box="box3")

    def pick_motion_2():
        '''
        Description: Goes to the picks and then drops 

        Args:

        Returns:

        ''' 

        nonlocal is_task2_completed
        nonlocal is_home_2
        nonlocal attached
        nonlocal to_pick

        # Do this task after timer_1 is cancelled
        if(timer_1.is_canceled()):
            if(is_home_2 == False):
                home_pose(home_flag_1)
                is_home_2 = True
                to_pick = True
            elif(is_home_2 == True):                
                if(to_pick == True):
                    try:
                        tr = tf_buffer.lookup_transform('base_link', 'obj_49', rclpy.time.Time())
                    except tf2_ros.TransformException as e:
                        node.get_logger().info(f'Could not transform base_link to obj_49: {e}')
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

                    # safety limits so that the rack does not move while manipulator goes to the drop config
                    dis_safe = 0.1

                    # coordinate errors
                    err_x = np.abs(x_des - x_pose)
                    err_y = np.abs(y_des - y_pose)
                    err_z = np.abs(z_des - z_pose)
                    err_rad = np.sqrt((x_des-x_pose)**2 + (y_des-y_pose)**2 + (z_des-z_pose)**2) 

                    # node.get_logger().info(f"x: {x_des}, y: %{y_des}, z: {z_des}\n")
                    node.get_logger().info(f"err_x: {err_x}, err_y: {err_y}, err_z: {err_z}, err_dis: {err_rad}\n")

                    # task two --> moving to pick-up point-2
                    if(is_task2_completed == False):
                        # moving to pre-pick position (in two steps --> have to change to single step)
                        if(err_x > tolerance):
                            moveit2_servo(linear=(x_des*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                        elif(err_z > tolerance):
                            moveit2_servo(linear=(0.0, 0.0, z_des*ssf), angular=(0.0, 0.0, 0.0))
                        # moving to pickup point-2
                        elif(err_y > tolerance):
                            moveit2_servo(linear=(0.0, y_des*ssf, 0.0), angular=(0.0, 0.0, 0.0))
                        elif(err_x <= tolerance and err_y <= tolerance and err_z <= tolerance):
                            moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                            is_task2_completed = True
                # if task-2 completed then go to home joint config and then go to drop off joint config
                    elif(is_task2_completed == True):
                        attached = send_attach_request("box49")
                        node.get_logger().info(f"{attached}")
                        if(attached == True):
                            if(np.abs(dis_safe-y_pose) > tolerance_2):
                                moveit2_servo(linear=(0.0, dis_safe*ssf, 0.0), angular=(0.0, 0.0, 0.0))
                            elif(np.abs(dis_safe-y_pose) <= tolerance_2):
                                moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                                to_pick = False
                else:
                    drop_motion(timer_=timer_2, home_flag_=home_flag_2, joint_flag_=joint_flag_2, box="box49")


    def pick_motion_3():
        '''
        Description: Goes to the picks and then drops 

        Args:

        Returns:

        ''' 
        nonlocal is_task3_completed
        nonlocal is_home_3
        nonlocal attached
        nonlocal to_pick

        if(timer_1.is_canceled() and timer_2.is_canceled()):
            if(is_home_3 == False):
                home_pose(home_flag_2)
                is_home_3 = True
                to_pick = True
            elif(is_home_3 == True):
                if(to_pick == True):
                    try:
                        tr = tf_buffer.lookup_transform('base_link', 'obj_1', rclpy.time.Time())
                    except tf2_ros.TransformException as e:
                        node.get_logger().info(f'Could not transform base_link to obj_1: {e}')
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

                    # safety limits so that the rack does not move while manipulator goes to the drop config
                    dis_safe = 0.15

                    # coordinate errors
                    err_x = np.abs(x_des - x_pose)
                    err_y = np.abs(y_des - y_pose)
                    err_z = np.abs(z_des - z_pose)
                    err_rad = np.sqrt((x_des-x_pose)**2 + (y_des-y_pose)**2 + (z_des-z_pose)**2)

                    # task three --> moving to pick-up point-3
                    if(is_task3_completed == False):
                        # moving to pre-pick position (in two steps --> have to change to single step)
                        if(err_x > tolerance):
                            moveit2_servo(linear=(x_des*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                        elif(err_z > tolerance):
                            moveit2_servo(linear=(0.0, 0.0, z_des*ssf), angular=(0.0, 0.0, 0.0))
                        # moving to pickup point-3
                        elif(err_y > tolerance):
                            moveit2_servo(linear=(0.0, y_des*ssf, 0.0), angular=(0.0, 0.0, 0.0))
                        elif(err_x <= tolerance and err_y <= tolerance and err_z <= tolerance):
                            moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                            is_task3_completed = True
                    # if task-3 completed then go to home joint config and then go to drop off joint config
                    elif(is_task3_completed == True):
                        attached = send_attach_request("box1")
                        node.get_logger().info(f"{attached}")
                        if(attached == True):
                            # if(np.abs(dis_safe-x_pose) > tolerance_2 and np.abs(dis_safe-y_pose) > tolerance_2):
                            #     moveit2_servo(linear=(-dis_safe*ssf, dis_safe*ssf, 0.0), angular=(0.0, 0.0, 0.0))
                            # elif(np.abs(dis_safe-x_pose) <= tolerance_2 and np.abs(dis_safe-x_pose) <= tolerance_2):
                            #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                                to_pick = False
                else:
                    drop_motion(timer_=timer_3, home_flag_=home_flag_2, joint_flag_=joint_flag_1, box="box1") 



    # servo_node/servo_start client 
    servo_client = node.create_client(Trigger, "servo_node/start_servo")
    # waiting for service
    while not servo_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('service not available, waiting again...')
    # function to call the servo service 
    servo_response = call_servo()
    node.get_logger().info(f"{servo_response.success}")

    # gripper attach and detach client
    gripper_attach_client = node.create_client(AttachLink, "GripperMagnetON")
    while not gripper_attach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service not available, waiting again...')

    gripper_detach_client = node.create_client(DetachLink, "GripperMagnetOFF")
    while not gripper_detach_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service not available, waiting again...')



    # timer functions created to do tasks in a sequential manner and to avoid intermediate errors that came up when I ran movit2_joint pose immediately after servoing or vice versa
    timer_1 = node.create_timer(0.02, pick_motion_1)
    # timer_2 = node.create_timer(0.02, pick_motion_2)
    # timer_3 = node.create_timer(0.02, pick_motion_3)

    executor_2 = rclpy.executors.MultiThreadedExecutor(2) 
    executor_2.add_node(node)
    executor_2.spin()
    
    rclpy.shutdown()

    exit(0)



if __name__ == '__main__':
    main()