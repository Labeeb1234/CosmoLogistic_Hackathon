#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import numpy as np


class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        
        # subscriber to imu data
        self.imu_data_sub = self.create_subscription(Imu, 'imu', self.imu_data_callback ,10)


        # # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'start_docking', self.dock_control_callback, callback_group=self.callback_group)
        
        # Create a publisher for sending velocity commands to the robot
        self.twist_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned = False
        self.linear_dock = None
        self.orientation_dock = None

        # necessary variables global to the class attributes and function
        self.usrleft_value = 0.0
        self.usrright_value = 0.0
        self.robot_pose = [0.0, 0.0, 0.0]
        self.robot_angle_imu = 0.0

        # Rack pose/vicinity properties
        self.rack_angle = 0.0
        self.rack_id = None
        self.pre_dock_position = [0.5, 4.35]
    
        # required tolreance values for the goal pose
        self.angle_tolerance = 0.01
        self.linear_tolerance = 0.01

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
        self.controller_timer.cancel()


    def normalize_angle(self, angle): # within range of -pi to pi (optional)
        if(angle >= np.pi):
            angle = angle - 2*np.pi
        elif(angle <= -np.pi):
            angle = angle + 2*np.pi
        
        else:
            angle = angle
        
        return angle
    
    # custom function for taking avg of both imu and odom yaw angles
    def avg_yaw(self, yaw1, yaw2):
        avg_yaw = (yaw1+yaw2)/2
        print("yaw_avg: %f\n" %avg_yaw)
        
        return avg_yaw
    
    # function to return average value of ultrasonic distance from the obstacle (need improvements)
    def avg_ultransonic_distance(self, ult_r, ult_l):
        avg_ult_distance = (ult_l+ult_r)/2

        return avg_ult_distance

    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        roll,pitch, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

        # self.get_logger().info(f"x_pose: {self.robot_pose[0]}, y_pose: {self.robot_pose[1]}, yaw_pose: {self.robot_pose[2]}\n")

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range
        # self.get_logger().info(f"ultrasonic left: {self.usrleft_value}\n")

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
        # self.get_logger().info(f"ultrasonic right: {self.usrright_value}\n")

    def imu_data_callback(self, msg):
        imu_orientation_array = msg.orientation
        imu_orientation_list = [imu_orientation_array.x, imu_orientation_array.y, imu_orientation_array.z, imu_orientation_array.w]
        roll, pitch, yaw = euler_from_quaternion(imu_orientation_list)
        self.robot_angle_imu = yaw

        # self.get_logger().info(f"robot_yaw_imu: {yaw}\n")

    def controller_loop(self):
        twist_msg = Twist()

        if(self.is_docking == True):
            # P controller logic
            # desired docking pose
            desired_yaw = self.rack_angle
            desired_x_goal = self.pre_dock_position[0]
            desired_y_goal = self.pre_dock_position[1]
            # print("rack angle in radians: %f" %(desired_yaw))

            # current pose of the robot
            current_robot_yaw = self.avg_yaw(self.robot_angle_imu, self.robot_pose[2])
            current_robot_x_pose = self.robot_pose[0]
            current_robot_y_pose = self.robot_pose[1]
            print("x_pose: %f, y_pose: %f, yaw_pose: %f\n" %(current_robot_x_pose, current_robot_y_pose, current_robot_yaw))

            # PID gains for yaw and linear motions
            kp_yaw = 1.0
            kp_linear = 0.1

            # global error values
            err_x_global = desired_x_goal-current_robot_x_pose 
            err_y_global = desired_y_goal-current_robot_y_pose
            err_yaw_global = desired_yaw-current_robot_yaw
            print("err_x: %f, err_y: %f, err_yaw: %f\n" %(err_x_global, err_y_global, err_yaw_global))
        
            # local errors (need to be given as twist cmds and twist msgs are robot frame velocities not global frame velocity) --> checked working logic
            err_x_local = math.cos(current_robot_yaw)*err_x_global + math.sin(current_robot_yaw)*err_y_global
            err_yaw_local = err_yaw_global
            
            # setting controlled robot frame velocities
            twist_msg.linear.x = kp_linear*err_x_local
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = kp_yaw*err_yaw_local
            # apply brakes after reaching the desired docking pose
            if(err_yaw_global < self.angle_tolerance):
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = 0.0
                self.dock_aligned = True
                self.is_docking = False

        
            self.twist_cmd_pub.publish(twist_msg)

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.linear_dock = request.linear_dock
        self.orientation_dock = request.orientation_dock
        self.rack_angle = request.orientation
        self.rack_id = request.rack_no

        self.get_logger().info(f"linear_dock: {self.linear_dock}, orientation_dock: {self.orientation_dock}, orientation: {self.rack_angle}, rack_id: {self.rack_id}\n")

        # Reset flags and start the docking process
        self.is_docking = True
        if(self.is_docking == True):
            self.controller_timer.reset()
        elif(self.is_docking == False):
            self.controller_timer.is_canceled()
            self.dock_control_srv.destroy()

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking done"
        return response
        


# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
