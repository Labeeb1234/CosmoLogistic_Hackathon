#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""


from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
# from pymoveit2 import MoveIt2Servo
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# my imports
from pymoveit2 import MoveIt2Servo
import numpy as np

# custom function to convert quaternions to euler angles
def quaternion_to_euler(z, x, y, w):
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

# Initialize message based on passed arguments

__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()

flag = True


def main():
    rclpy.init()
 
    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    # Create MoveIt 2 Servo interface
    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id= __twist_msg.header.frame_id,
        callback_group=callback_group,
    )

    tf_buffer = tf2_ros.buffer.Buffer()
    
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # twist msg callback function
    def servo_circular_motion():
        """Move in a circular motion using Servo"""
        current_time = node.get_clock().now().nanoseconds*1e-9

        try:
            tr = tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
        except tf2_ros.TransformException as e:
            node.get_logger().info(f'Could not transform base_link to tool0: {e}')
            return
        
        # pose of the EEF
        x_pose = tr.transform.translation.x
        y_pose = tr.transform.translation.y
        z_pose = tr.transform.translation.z
            # angles
        x_orient = tr.transform.rotation.x
        y_orient = tr.transform.rotation.y
        z_orient = tr.transform.rotation.z
        w_orient = tr.transform.rotation.w

        print("x: %f, y: %f, z: %f\n" %(x_pose, y_pose, z_pose))
        print("x: %f, y: %f, z: %f, w: %f\n" %(x_orient, y_orient, z_orient, w_orient))
    
        linear_x = 0.35/10
        linear_y = 0.1/10
        linear_z = 0.68/10
        # linear_x = 0.194/5 
        # linear_y = -0.43/5
        # linear_z = 0.701/5

        angular_y = np.pi/2

        W, X, Y, Z = euler_to_quaternion(0.0, np.pi/2, 0.0)


        
        moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, -angular_y, 0.0))


        # # pick up pose P1
        # if(y_pose > 0.1):
        #     moveit2_servo(linear=(linear_x, -linear_y, linear_z), angular=(0.0, 0.0, 0.0))
        # elif(x_pose <= 0.35 and y_pose <= 0.1 and z_pose <= 0.68):
        #     moveit2_servo(linear=(linear_x, 0.0, linear_z), angular=(0.0, 0.0, 0.0))
        # elif(x_pose <= 0.35):
        #     moveit2_servo(linear=(linear_x, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
        # elif(x_pose > 0.35 and y_pose < 0.1 and z_pose > 0.68):
        #     moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))


    # Create timer for moving in a circular motion
    node.create_timer(0.02, servo_circular_motion)
    

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()


'''
if(frameId = base_link)
linear vel:
x --> EEF moves along red axis of base_link TF (x-axis)
y --> EEF moves along green axis of the base_link TF (y-axis)
z --> EEF moves along blue axis of the base_link TF (z-axis)

angular vel:
x --> EEF rotates about the red axis of the base_link TF (x-axis)
y --> EEF rotates about the green axis of the base_link TF (y-axis)
z --> EEF rotates about the blue axis of the base_link TF (z-axis)

'''


