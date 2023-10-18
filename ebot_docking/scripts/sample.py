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
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.twist_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize all  flags and parameters here
        self.is_docking = True
        self.dock_aligned = False

        # required tolreance values for the goal pose
        self.angle_tolerance = 0.01
        self.linear_tolerance = 0.01

        self.usrleft_value = 0.0
        self.usrright_value = 0.0

        self.robot_pose = [0.0, 0.0, 0.0]

        self.rack1_pose  = [1.260008, 4.350000, 3.14]
        self.goal_pose = [0.5, 4.35, 3.14]

        self.robot_angle_imu = 0.0
        self.desired_angle = 0.0
        
    

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)


    def normalize_angle(self, angle): # within range of -pi to pi (optional)
        if(angle >= np.pi):
            angle = angle - 2*np.pi
        elif(angle <= -np.pi):
            angle = angle + 2*np.pi
        
        else:
            angle = angle
        
        return angle
    
    def avg_yaw(self, yaw1, yaw2):
        avg_yaw = (yaw1+yaw2)/2
        print("yaw_avg: %f\n" %avg_yaw)
        
        return avg_yaw
    
    def avg_ultransonic_distance(self, ult_r, ult_l):
        avg_ult_distance = (ult_l+ult_r)/2

        return avg_ult_distance

    def avg_pose(self, position):
        pass

    def err_glob_to_loc(self, err_local, err_global):
        pass


    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        roll , pitch, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw
        # self.get_logger().info(f"x_pose: {self.robot_pose[0]}, y_pose: {self.robot_pose[1]}, yaw_pose: {self.robot_pose[2]}\n")

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range
        self.get_logger().info(f"ultrasonic left: {self.usrleft_value}\n")

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
        self.get_logger().info(f"ultrasonic right: {self.usrright_value}\n")

    def imu_data_callback(self, msg):
        
        imu_orientation_array = msg.orientation
        imu_orientation_list = [imu_orientation_array.x, imu_orientation_array.y, imu_orientation_array.z, imu_orientation_array.w]
        roll, pitch, yaw = euler_from_quaternion(imu_orientation_list)
        self.robot_angle_imu = yaw

        # self.get_logger().info(f"robot_yaw_imu: {yaw}\n")

    def controller_loop(self):
        twist_msg = Twist()

        # P controller logic
        desired_rack_angle = self.rack1_pose[2]
        desired_rack_angle = self.desired_angle
        desired_rack_y_pose = self.rack1_pose[1]
        desired_ult_distance = 0.1

        current_robot_yaw = self.avg_yaw(self.robot_angle_imu, self.robot_pose[2])
        current_robot_x_pose = self.robot_pose[0]
        current_robot_y_pose = self.robot_pose[1]

        # PID gains for yaw and linear motions
        kp_yaw = 1.0
        kp_linear = 0.1

        # global error values
        err_x_global = self.goal_pose[0]-current_robot_x_pose
        err_y_global = self.goal_pose[1]-current_robot_y_pose
        err_yaw_global = self.goal_pose[2]-current_robot_yaw
        
        # local errors (need to be given as twist cmds and twist msgs are robot frame velocities not global frame velocity) --> checked working logic
        err_x_local = math.cos(current_robot_yaw)*err_x_global + math.sin(current_robot_yaw)*err_y_global
        err_yaw_local = err_yaw_global

        print("err_x: %f, err_y: %f, err_yaw: %f\n" %(err_x_global, err_y_global, err_yaw_global))
        # Note: no safety limits given here
        if(self.is_docking == True):
            twist_msg.linear.x = kp_linear*err_x_local
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = kp_yaw*err_yaw_local
            if(err_x_global < self.linear_tolerance and err_y_global < self.linear_tolerance and err_yaw_global < self.angle_tolerance):
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.angular.z = 0.0
                self.is_docking = False
                self.dock_aligned = True
        # apply brakes after dock alignment
        elif(self.is_docking == False):
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0
            self.dock_aligned = True
            self.controller_timer.cancel()
        
        
        self.twist_cmd_pub.publish(twist_msg)

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        request = DockSw().Request()
        need_linear_dock = request.linear_dock
        need_orientation_dock = request.orientation_dock
        self.desired_angle  = request.orientation
        rack_id = request.rack_no

        # Reset flags and start the docking process
        self.is_docking = True
        if(need_linear_dock == True and need_orientation_dock == True):
            self.controller_timer.is_ready()

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
        response.message = "Docking control initiated"
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
