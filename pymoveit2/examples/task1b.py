#!/usr/bin/env python3


# Team ID:          [ Team-ID: 3166 ]
# Author List:		[Team Members: Labeeb, Amar, Abhinand, Dheeraj]
# Filename:		    task1b.py
# Functions:        [custom functions used: home_pose, go_to_pose, servo_move_1, servo_move_2, servo_move_3, servo_move_4]
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

# variable tags for twist_stamped msgs to be used for servoing control
__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()

def main():
    '''
    This node is uses both movit2 and servoing method in combinations to implement a simulation pick-and-place-ish kind of actions.
    Both methods are run on the same node in sequence of actions.
    This method uses the servo to increase the accuracy of each coordinate the end effector reaches,
    hence, there is breakdown in the x-y-z velocity cmds.
    --> alternatively we can put into to single velocity vector to get one smooth motion to reach the required end-effector points,
        but the accuracy reduces  
    '''

    # global timer function variables declared use to assign to each create_timer functions 
    global timer_1
    global timer_2
    global timer_3
    global timer_4
    global servo_client
    
    rclpy.init()
    node_2 = Node("servo_goal")

    # global request
    # request = Trigger().Request()
    
    # tf_ros.buffer is used for looking up transform between base_link and tool0 (located at the end-effector)
    tf_buffer = tf2_ros.buffer.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node_2)
    
    # variables and flags or importance of this node action
    drop = False
    # flags used the home_pose() and go_to_pose() custom fucntions (given below)
    home_flag_1 = 1
    home_flag_2 = 0
    joint_flag_1 = 1
    joint_flag_2 = 0
    # set of flags to indicate the completion of individual subtasks that I divided the manipulator motions into
    is_task1_completed = False
    is_task2_completed = False
    is_task3_completed = False
    is_task4_completed = False
    tolerance = 0.001 # controlling the error level in the manipluator end effector positions
    ssf = 1/8.0 # speed scaling factor


    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node_2,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    moveit2_servo = MoveIt2Servo(
        node=node_2,
        frame_id= __twist_msg.header.frame_id,
        callback_group=callback_group,
    )

    def call_servo():
        request = Trigger.Request()
        node_2.get_logger().info("Servo_node service started\n")
        future = servo_client.call_async(request)
        rclpy.spin_until_future_complete(node_2, future)
    
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
        # joint_position_3 = [-0.0009985368154801222 ,-1.5704905644661782, -1.5702299301173275, -3.1395326273635997, -1.5700358961661465, -0.0001215935494314202]
        joint_position_2 = [0.0 ,-2.390015399788957, 2.4000294436697867, -3.150012833918351, -1.5799936049581742, 3.150000000942502]
        
        if(joint_config == 1):
            moveit2.move_to_configuration(joint_positions=joint_position_1)
            moveit2.wait_until_executed()
        elif(joint_config == 0):
            moveit2.move_to_configuration(joint_positions=joint_position_2)
            moveit2.wait_until_executed()
        
        # timer_1.cancel()

    def servo_move_1():
        '''
        Description:    Goes to the first pick up point and orients the manipulator to the drop-off point joint

        Args:

        Returns:

        '''
        global timer_1
        current_time = (node_2.get_clock().now().nanoseconds)*1e-9
        nonlocal is_task1_completed
        # if(servo_client.service_is_ready()):
        try:
            tr = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
        except tf2_ros.TransformException as e:
            node_2.get_logger().info(f'Could not transform base_link to tool0: {e}')
            return
        
        # looked up pose of the EEF wrt base_link of manipulator
        x_pose = tr.transform.translation.x
        y_pose = tr.transform.translation.y
        z_pose = tr.transform.translation.z

        # coordinate errors
        err_x = np.abs(0.35 - x_pose)
        err_y = np.abs(0.1 - y_pose)
        err_z = np.abs(0.68 - z_pose)
        # calcualted distanced and desired distance the end-effector vector is supposed to reach (withing the requied error limits)
        desired_distance = np.sqrt(0.35**2 + 0.1**2 + 0.68**2)
        current_distance = np.sqrt((x_pose)**2 + (y_pose)**2 + (z_pose)**2)

        # err_radius = np.sqrt((x_pose - 0.35)**2 + (y_pose - 0.1)**2 + (z_pose - 0.68)**2)
        print("x: %f, y: %f, z: %f\n" %(x_pose, y_pose, z_pose))
        print("current_dis: %f, desired_dis: %f\n" %(current_distance, desired_distance))
        print("error_distance: %f\n" %(np.abs(current_distance-desired_distance)))
        # print("distance between vectors: %f\n" %(err_radius))

        # task one --> moving to pick-up point-1
        if(is_task1_completed == False):
            # if(err_x > tolerance):
            #     moveit2_servo(linear=(0.35*ssf, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
            # elif(err_y > tolerance):
            #     moveit2_servo(linear=(0.0, 0.1*ssf, 0.0), angular=(0.0, 0.0, 0.0))
            # elif(err_z > tolerance):
            #     moveit2_servo(linear=(0.0, 0.0, 0.68*ssf), angular=(0.0, 0.0, 0.0))
            # elif(err_x <= tolerance and err_y <= tolerance and err_z <= tolerance):
            #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
            #     is_task1_completed = True
            if(np.abs(current_distance-desired_distance) > tolerance):
                moveit2_servo(linear=(0.35*ssf, 0.1*ssf, 0.68*ssf), angular=(0.0, 0.0, 0.0))
            elif(np.abs(current_distance-desired_distance) <= tolerance):
                moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                is_task1_completed = True
        # if task-1 completed then go to home joint config and then go to drop off joint config
        elif(is_task1_completed == True):
            home_pose(home=home_flag_1)
            go_to_pose(joint_config=joint_flag_1)
            # stop timer thereafter 
            timer_1.cancel()

    def servo_move_2():
        '''
        Description:   Orients the end effector to the correct drop off point and then orients itself to the alternative home joint configuration 

        Args:

        Returns:

        ''' 
        global timer_2
        nonlocal is_task2_completed

        # Do this task after timer_1 is cancelled 
        if(timer_1.is_canceled()):
            try:
                tr = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            except tf2_ros.TransformException as e:
                node_2.get_logger().info(f'Could not transform base_link to tool0: {e}')
                return
            
            # looked up pose of the EEF wrt base_link of manipulator
            x_pose = tr.transform.translation.x
            y_pose = tr.transform.translation.y
            z_pose = tr.transform.translation.z
            # drop --> [-0.37, 0.12, 0.397] 

            # coordinate errors
            err_x = np.abs(-0.37 - x_pose)
            err_y = np.abs(0.12 - y_pose)
            err_z = np.abs(0.397 - z_pose)

            desired_distance = np.sqrt(0.37**2 + 0.12**2 + 0.397**2)
            current_distance = np.sqrt(x_pose**2 + y_pose**2 + z_pose**2)

            print("x: %f, y: %f, z: %f\n" %(x_pose, y_pose, z_pose))
            print("current_dis: %f, desired_dis: %f\n" %(current_distance, desired_distance))
            print("error_distance: %f\n" %(np.abs(current_distance-desired_distance)))
            
            # task two --> going to drop-off point
            if(is_task2_completed == False):
                # if(err_x > tolerance):
                #     moveit2_servo(linear=(0.37, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_y > tolerance):
                #     moveit2_servo(linear=(0.0, 0.12, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_z > tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, -0.397), angular=(0.0, 0.0, 0.0))
                # elif(err_x <= tolerance, err_y <= tolerance and err_z <= tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                #     is_task2_completed = True
                if(np.abs(current_distance-desired_distance) > tolerance):
                    moveit2_servo(linear=(0.37*ssf, 0.12*ssf, -0.397*ssf), angular=(0.0, 0.0, 0.0))
                elif(np.abs(current_distance-desired_distance) <= tolerance):
                    moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                    is_task2_completed = True
            # if task-2 is completed then go to the alternative home pose to go to pick-up point-2
            elif(is_task2_completed == True):
                home_pose(home=home_flag_2)
                # stop timer thereafter 
                timer_2.cancel()

    def servo_move_3():
        '''
        Description:   Goes to the second pick up point and orients the manipulator to the alternative home joint configuration 

        Args:

        Returns:

        ''' 
        global timer_3
        nonlocal is_task3_completed
        # Do this task after timer_1 and timer_2 is cancelled
        if(timer_1.is_canceled() and timer_2.is_canceled()):
            try:
                tr = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            except tf2_ros.TransformException as e:
                node_2.get_logger().info(f'Could not transform base_link to tool0: {e}')
                return
            
            # looked up pose of the EEF wrt base_link of manipulator
            x_pose = tr.transform.translation.x
            y_pose = tr.transform.translation.y
            z_pose = tr.transform.translation.z
            #     # angles
            # x_orient = tr.transform.rotation.x
            # y_orient = tr.transform.rotation.y
            # z_orient = tr.transform.rotation.z 
            # w_orient = tr.transform.rotation.w
            # pick point-2 [0.194, -0.43, 0.701]

            # coordinate errors
            err_x = np.abs(0.194 - x_pose)
            err_y = np.abs(-0.43 - y_pose)
            err_z = np.abs(0.701 - z_pose)

            desired_distance = np.sqrt(0.194**2 + 0.43**2 + 0.701**2)
            current_distance = np.sqrt(x_pose**2 + y_pose**2 + z_pose**2)

            print("x: %f, y: %f, z: %f\n" %(x_pose, y_pose, z_pose))
            print("current_dis: %f, desired_dis: %f\n" %(current_distance, desired_distance))
            print("error_distance: %f\n" %(np.abs(current_distance-desired_distance)))
            # task_3 --> moving to pick-up point-2
            if(is_task3_completed == False):
                # if(err_x > tolerance):
                #     moveit2_servo(linear=(0.194, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_y > tolerance):
                #     moveit2_servo(linear=(0.0, -0.43, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_z > tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, 0.701), angular=(0.0, 0.0, 0.0))
                # elif(err_x <= tolerance and err_y <= tolerance and err_z <= tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                #     is_task3_completed = True
                if(np.abs(current_distance-desired_distance) > tolerance):
                    moveit2_servo(linear=(0.194*ssf, -0.43*ssf, 0.701*ssf), angular=(0.0, 0.0, 0.0))
                elif(np.abs(current_distance-desired_distance) <= tolerance):
                    moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                    is_task3_completed = True
            # if task-3 is completed then go to the alternative home pose and go to drop off joint config 
            elif(is_task3_completed == True):
                home_pose(home=home_flag_2)
                go_to_pose(joint_config=joint_flag_1)
                # stop timer thereafter
                timer_3.cancel()
    
    def servo_move_4():
        '''
        Description:    Orients the end effector to the correct drop off point and then orients itself to the default home joint configuration 

        Args:

        Returns:

        ''' 
        global timer_4
        nonlocal is_task4_completed 
        # Do this task after timer_1 and timer_2 and timer_3 is cancelled
        if(timer_1.is_canceled() and timer_2.is_canceled() and timer_3.is_canceled()):
            try:
                tr = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            except tf2_ros.TransformException as e:
                node_2.get_logger().info(f'Could not transform base_link to tool0: {e}')
                return
            
            # looked up pose of the EEF wrt base_link of manipulator
            x_pose = tr.transform.translation.x
            y_pose = tr.transform.translation.y
            z_pose = tr.transform.translation.z
            #     # angles
            # x_orient = tr.transform.rotation.x
            # y_orient = tr.transform.rotation.y
            # z_orient = tr.transform.rotation.z
            # w_orient = tr.transform.rotation.w
            # drop --> [-0.37, 0.12, 0.397] 

            # coordinate errors
            err_x = np.abs(-0.37 - x_pose)
            err_y = np.abs(0.12 - y_pose)
            err_z = np.abs(0.397 - z_pose)

            desired_distance = np.sqrt(0.37**2 + 0.12**2 + 0.397**2)
            current_distance = np.sqrt(x_pose**2 + y_pose**2 + z_pose**2)

            print("x: %f, y: %f, z: %f\n" %(x_pose, y_pose, z_pose))
            print("current_dis: %f, desired_dis: %f\n" %(current_distance, desired_distance))
            print("error_distance: %f\n" %(np.abs(current_distance-desired_distance)))
            # task-4 --> go to the drop-off point
            if(is_task4_completed == False):
                # if(err_x > tolerance):
                #     moveit2_servo(linear=(0.37, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_y > tolerance):
                #     moveit2_servo(linear=(0.0, 0.12, 0.0), angular=(0.0, 0.0, 0.0))
                # elif(err_z > tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, -0.397), angular=(0.0, 0.0, 0.0))
                # elif(err_x <= tolerance, err_y <= tolerance and err_z <= tolerance):
                #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                #     is_task4_completed = True
                if(np.abs(current_distance-desired_distance) > tolerance):
                    moveit2_servo(linear=(0.37*ssf, 0.12*ssf, -0.397*ssf), angular=(0.0, 0.0, 0.0))
                elif(np.abs(current_distance-desired_distance) <= tolerance):
                    moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
                    is_task4_completed = True
            # if task-4 is completed go to the default home position
            elif(is_task4_completed == True):
                home_pose(home=home_flag_1)
                # stop timer thereafter
                timer_4.cancel()
        
    # timer functions created to do tasks in a sequential manner and to avoid intermediate errors that came up when I ran movit2_joint pose immediately after servoing or vice versa 
    servo_client = node_2.create_client(Trigger, "/servo_node/start_servo")
    while not servo_client.wait_for_service(timeout_sec=10.0):
        node_2.get_logger().info('service not available, waiting again...')
    call_servo()

    timer_1 = node_2.create_timer(0.02, servo_move_1)
    timer_2 = node_2.create_timer(0.02, servo_move_2)
    timer_3 = node_2.create_timer(0.02, servo_move_3)
    timer_4 = node_2.create_timer(0.02, servo_move_4)

    executor_2 = rclpy.executors.MultiThreadedExecutor(2)
    executor_2.add_node(node_2)
    executor_2.spin()
    
    rclpy.shutdown()

    exit(0)



if __name__ == '__main__':
    main()