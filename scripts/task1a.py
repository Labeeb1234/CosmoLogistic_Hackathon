#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID: 3166 ]
# Author List:		[Team Members: Labeeb, Amar, Abhinand, Dheeraj ]
# Filename:		    task1a.py
# Functions:
#			        [euler_to_quaternion(custom),  calculate_rectangle_area, detect_aruco, Class functions: depthimagecb, colorimagecb, process_image]
# Nodes:		    publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /camera/color/image_raw]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import quaternion_from_euler


##################### FUNCTION DEFINITIONS #######################
# adding an extra custom function to convert euler angles to quaternions
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
    return x, y, z, w

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    # area = None
    # width = None

    ############ ADD YOUR CODE HERE ############
    width = (np.abs(np.linalg.norm(coordinates[0]-coordinates[1])) + np.abs(np.linalg.norm(coordinates[2]-coordinates[3])))/2
    height_marker_1 = (np.abs(np.linalg.norm(coordinates[1]-coordinates[2])) + np.abs(np.linalg.norm(coordinates[1]-coordinates[2])))/2

    area = 0.5*np.abs((coordinates[0][0]*coordinates[1][1]-coordinates[0][1]*coordinates[1][0])+
                      (coordinates[1][0]*coordinates[2][1]-coordinates[2][0]*coordinates[1][1])+
                      (coordinates[2][0]*coordinates[3][1]-coordinates[3][0]*coordinates[2][1])+
                      (coordinates[3][0]*coordinates[0][1]-coordinates[0][0]*coordinates[3][1])
                      )

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    # ids = []
 
    ############ ADD YOUR CODE HERE ############
    # extra variable 
    Area = [] # list for storing area values
    rvecs = [] # list for storing the rotation vectors of aruco markers
    tvecs = [] # list for storing the translation vectors of aruco markers


    bw_img = cv2.imread(image, 1)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    
    corners, id, rejected_img_points = cv2.aruco.detectMarkers(bw_img, arucoDict, parameters=parameters, cameraMatrix=cam_mat, distCoeff=dist_mat)

    try:
        id = id.flatten()
    except AttributeError:
        pass

    if id is not None and len(id) > 0:
       
        # to draw over all the detected aruco markers
        cv2.aruco.drawDetectedMarkers(bw_img, corners, id) 
        
        # id = np.sort(id) # sorted id list
        corner_coordinates = []
        for i in range(0, len(corners)):
            corner_coordinates.append(corners[i])

        for i in range(0, len(id)):
            corner_coordinates[i] = corner_coordinates[i].reshape((4,2))

        for i in range(0, len(id)):
             area_i, width_i = calculate_rectangle_area(corner_coordinates[i])
             Area.append(area_i)
             width_aruco_list.append(width_i)

        for i in range(0, len(id)):
            if(Area[i] < aruco_area_threshold):
                id = np.delete(id, i)
                corner_coordinates.pop(i)
                width_aruco_list.pop(i)
        
        for (markerid, corner) in zip(id, corners):
            rvecs_i, tvecs_i, _ = cv2.aruco.estimatePoseSingleMarkers(corner,size_of_aruco_m, cam_mat, dist_mat)
            rvecs.append(rvecs_i)
            tvecs.append(tvecs_i)

        # print(rvecs)

        for i in range(0, len(id)):
            center_aruco_list.append(np.mean(corner_coordinates[i], axis=0))        
        # handle missing cases of centre_coordinates(may have to delete)
        # for i in range(0, len(id)):
        #     if(len(center_aruco_list)==3):
        #         center_aruco_list[i] = center_aruco_list[i]
        #     elif(len(center_aruco_list) < 3):
        #         # calculated running avg/modal overrride
        #         center_aruco_list = [np.array([216.25, 405.75]), np.array([746.25, 376.5 ]), np.array([479., 375.5])]


        for i in range(0, len(id)): 
            rvecs[i] = rvecs[i].flatten()               
            distance_from_rgb_list.append(np.linalg.norm(tvecs[i]))
            angle_aruco_list.append(rvecs[i])
        
        # handling exceptions of missing distance_from_rgb and aruco angles(not required)
        for i in range(0, len(id)):
            if(len(distance_from_rgb_list) == 3):
                distance_from_rgb_list[i] = distance_from_rgb_list[i]
            elif(len(distance_from_rgb_list)<3):
                distance_from_rgb_list = [1.5069559335892873, 1.3562067508739548, 1.4279646687901528]

        
        
        for (markerid, rvec, tvec) in zip(id, rvecs, tvecs):
            bw_img = cv2.drawFrameAxes(bw_img, cam_mat, dist_mat, rvec, tvec, 1.0)

        # saving the transformed image files for further processing
        cv2.imwrite('transformed_image.jpg', bw_img)
    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################
    
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, id


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(data)
        cv2.imwrite('new.jpg', self.cv_image)

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############
        # defining some constants found from the pre-existing trasnformations between base_link and camera_link
        cam_frame_tilt = 0.2610002 # in Euler angles (measured with respect to base_link and is a constant)
        roll = 0.0
        pitch = 0.0
    
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, id = detect_aruco('new.jpg')
        image = cv2.imread('transformed_image.jpg', 1)

        # if(len(id) == 3):
            # angle correction (yaw)
        for i in range(0, len(angle_aruco_list)):
            angle_aruco_list[i][2] = (0.788*angle_aruco_list[i][2]) - ((angle_aruco_list[i][2]**2)/3160)

        try:
            # logging down the depth image values in [m] for each marker that is detected withing range
            for (markerid, centres) in zip(id, center_aruco_list):
                cX = centres[0]
                cY = centres[1]
                depth_mm = self.depth_image[int(cY), int(cX)]
                self.get_logger().info(f"Real sense depth of {markerid}: {depth_mm/1000}")

            # getting marker frames in Rviz and broadcasting them to the tf-tree of the system
            for (markerid, coordinates, angles) in zip(id, center_aruco_list, angle_aruco_list):
                
                cX = coordinates[0]
                cY = coordinates[1]
                # centre point marker on image put text too
                cv2.circle(image, (int(cX),int(cY)), 1, (255, 153, 255), 10)
                cv2.putText(image, "center", (int(cX)-30, int(cY)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 2)

                depth_image = self.depth_image[int(cY), int(cX)]
                real_sense_depth = depth_image/1000
                distance_from_rgb = real_sense_depth
                
                x = distance_from_rgb* (sizeCamX - cX - centerCamX) / focalX
                y = distance_from_rgb* (sizeCamY - cY - centerCamY) / focalY
                z = distance_from_rgb
                # self.get_logger().info(f"x: {x}, y: {y}, z: {z}")
            
                # angle correction for tilt for final TF
                if(markerid == 49):
                    # rotations on the side racks
                    # roll = np.pi/2
                    # pitch = cam_frame_tilt
                    # angles[2] = angles[2] - np.pi/2
                    roll = np.pi/2 - cam_frame_tilt
                    pitch = 0.0
                    angles[2] = angles[2] + np.pi/2
                elif(markerid == 3):
                    # middle rack rotations
                    roll = np.pi/2 - cam_frame_tilt
                    pitch = 0.0
                    angles[2] = angles[2] + np.pi/2
                else:
                    roll = np.pi/2
                    pitch = -cam_frame_tilt
                    angles[2] = angles[2] - np.pi/2

                quaternion = euler_to_quaternion(roll, pitch, angles[2])
                transform_stamped = TransformStamped()
                transform_stamped.header.frame_id = 'camera_link'
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.child_frame_id = 'cam_'+ str(markerid)
                transform_stamped.transform.translation.x = z 
                transform_stamped.transform.translation.y = x
                transform_stamped.transform.translation.z = y 
                transform_stamped.transform.rotation.x = quaternion[0]
                transform_stamped.transform.rotation.y = quaternion[1]
                transform_stamped.transform.rotation.z = quaternion[2]
                transform_stamped.transform.rotation.w = quaternion[3]
                self.br.sendTransform(transform_stamped)

                # making a look up between objects and base_link for further transformation between base_link and objects
                from_frame_rel = 'cam_'+str(markerid)                                                                      
                to_frame_rel = 'base_link'                                                                  

                try:
                    transformed_tr = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())  
                    # self.get_logger().info(f'Successfully received data!')
                except tf2_ros.TransformException as e:
                    self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
                    return

                x = transformed_tr.transform.translation.x 
                y = transformed_tr.transform.translation.y
                z = transformed_tr.transform.translation.z 
                Z = transformed_tr.transform.rotation.z
                X = transformed_tr.transform.rotation.x 
                Y = transformed_tr.transform.rotation.y 
                W = transformed_tr.transform.rotation.w 

                ts = TransformStamped()
                ts.header.frame_id = 'base_link'
                ts.header.stamp = self.get_clock().now().to_msg()
                ts.child_frame_id = 'obj_' + str(markerid)
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                ts.transform.rotation.z = Z 
                ts.transform.rotation.x = X 
                ts.transform.rotation.y = Y 
                ts.transform.rotation.w = W 
                self.br.sendTransform(ts)

            # image displayed
            cv2.imshow('task-1a', image)
            cv2.waitKey(1)
        except TypeError:
            pass
        else:
            pass
        
        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()