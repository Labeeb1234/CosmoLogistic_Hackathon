#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[-0.37, 0.12, 0.397]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""
# 0.505408, y: 0.496698, z: 0.498705, w: 0.499146
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
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
# x: 0.187259, y: 0.107824, z: 0.634850

    # Create node for this example
    node = Node("ex_pose_goal")

    w, x, y, z = euler_to_quaternion(0.0, 0.0, 0.0)

    # Declare parameters for position and orientation
    node.declare_parameter("position", [-0.37, 0.12, 0.397])
    node.declare_parameter("quat_xyzw", [x, y, w, z])
    node.declare_parameter("cartesian", True)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM, 
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
