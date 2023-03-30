#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import tf.transformations
import tf2_ros
from kortex_driver.msg import BaseCyclic_Feedback

########################
camera_x = None
camera_y = None
camera_z = None
c_b_x = 0
c_b_y = 0
c_b_z = 0
pixel_x = 0
pixel_y = 0
height = 200
gripper_current = 0
gripper_position = 0 # fully open
mode = 0
miss = 0
########################
def callback_miss(data):
    global miss
    miss = data.data

def callback_mode(data):
    global mode
    mode = data.data

def callback_motor(data):
    global gripper_current, gripper_position
    gripper_current = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].current_motor)
    gripper_position = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)

def callback_camera(data):
    global camera_x, camera_y, camera_z
    global pixel_x, pixel_y
    global c_b_x, c_b_y, c_b_z

    camera_x = data.position.x
    camera_y = data.position.y
    camera_z = data.position.z

    c_b_x = 0.47 * camera_z - 0.88 * camera_y - 0.1
    c_b_y = -camera_x - 0.25
    c_b_z = -0.88 * camera_z - 0.47 * camera_y + 0.85

    # ------- transfer from webcam coordinate to image plane -------
    pixel_x = (611.055419921875*camera_x/camera_z) + 316.6255187988281 # get it by $rostopic echo /camera/color/camera_info
    pixel_y = (611.2431030273438*camera_y/camera_z) + 229.1320343017578 # get it by $rostopic echo /camera/color/camera_info
    #pixel_x = int(pixel_x)
    #pixel_y = int(pixel_y)
    # ------- ---------------------------------------------- -------

def callback_image(data):
    global rgb_img
    rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

###############################################################
rospy.Subscriber('/camera/color/image_raw', Image, callback_image)
###############################################################

if __name__ == '__main__':
    global pixel_x, pixel_y, rgb_img
    global c_b_x, c_b_y, c_b_z
    global gripper_current, gripper_position
    global mode, miss
    rospy.init_node('image_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)

    i = 0

    while not rospy.is_shutdown():
        
        # get the status of the gripper
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, callback_motor)
        #print(gripper_position)

        # get the object location w.r.t. image plane: (pixel_x, pixel_y)
        rospy.Subscriber('/aruco_simple/pose2', Pose, callback_camera)
        #print(pixel_x, pixel_y)

        # get the status of controller button
        rospy.Subscriber('/mode_topic', Float64, callback_mode, queue_size=1)
        #print(mode)

        # get the status of visual secondary task
        rospy.Subscriber('/miss_topic', Float64, callback_miss, queue_size=1)

        # get the robot end effector w.r.t. image plane: (pixel_x_r, pixel_y_r)
        robot_ee = tfBuffer.lookup_transform('base_link', 'robotiq_arg2f_base_link', rospy.Time())
        # ------- transfer from robot base frame to webcam coordinate ------- 
        ee_x = robot_ee.transform.translation.x
        ee_y = robot_ee.transform.translation.y
        ee_z = robot_ee.transform.translation.z
        ee_cam_x = -ee_y - 0.29 #ee_cam_x = ee_y + 0.25 # Cx = -Ry - (Ry_offset)
        ee_cam_y = -0.966 * ee_x - 0.259 * ee_z + 0.26 #ee_cam_y = 0.88 * ee_x + 0.47 * ee_z - 0.35 #(0.47 * 0.85 + 0.88 * 0.1) # Cy = -Rx*sin()-Rz*cos() + ( Rz_offset*cos() + Rx_offset*sin() )
        ee_cam_z = 0.259 * ee_x - 0.966 * ee_z + 0.9 #ee_cam_z = -0.47 * ee_x + 0.88 * ee_z - 0.7 #(0.88 * 0.85 - 0.47 * 0.1) # Cz = Rx*cos()-Rz*sin() - ( Rz_offset*sin() - Rx_offset*cos() )
        # ------- --------------------------------------------------- ------- 

        # ------- transfer from realsense coordinate to image plane -------
        pixel_x_r = (611.055419921875*ee_cam_x/ee_cam_z) + 316.6255187988281 # get it by $rostopic echo /camera/color/camera_info
        pixel_y_r = (611.2431030273438*ee_cam_y/ee_cam_z) + 229.1320343017578 # get it by $rostopic echo /camera/color/camera_info
        #pixel_x_r = int(pixel_x_r)
        #pixel_y_r = int(pixel_y_r)
        # ------- ---------------------------------------------- -------

        # ------- calculate distance and line equation -------
        distance = math.hypot(pixel_x_r - (pixel_x), pixel_y_r - pixel_y)
        #print(distance)
        distance_m = math.sqrt((c_b_x - ee_x) ** 2 + (c_b_y - ee_y) ** 2 + (c_b_z - ee_z) ** 2)
        distance_cm = distance_m*100
        #print(distance_m)
        slope = (pixel_y - pixel_y_r) / (pixel_x - pixel_x_r)
        pixel_y_r_xout_0 = slope*(15 - pixel_x_r) + pixel_y_r
        pixel_x_r_yout_0 = (15 - pixel_y_r) / slope + pixel_x_r
        pixel_y_r_xout_640 = slope*(625 - pixel_x_r) + pixel_y_r
        pixel_x_r_yout_480 = (465 - pixel_y_r) / slope + pixel_x_r
        # ------- ------------------ -------

        # ####### create UI for ar interface #######
        
        # ------- Check the completion of grasping -------
        if gripper_position > 28 and int(height) > 415 and distance < 13:
            Grasp_complete_text = cv2.rectangle(rgb_img, (185, 420), (475, 390), (255, 255, 255), cv2.FILLED)
            Grasp_complete_text_bg = cv2.putText(rgb_img, 'Grasping Completed', (190, 412), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            cv2.namedWindow('AR Interface', cv2.WINDOW_GUI_NORMAL) # or can use cv2.WINDOW_KEEPRATIO in the last parameter
            cv2.resizeWindow('AR Interface', 1280, 960)
            cv2.imshow('AR Interface', Grasp_complete_text)
            key = cv2.waitKey(1)
        elif gripper_position < 28:
        # ------- Check the completion of grasping -------

            Hint1_location_x = (-10, -10)
            Hint1_location_y = (-10, -10)
            Hint2_location_x = (-10, -10)
            Hint2_location_y = (-10, -10)
            # ------- object location -------
            if pixel_x < 640 and pixel_y < 480:
                object_location = (int(pixel_x), int(pixel_y))
                object_location_color = (255, 0 ,255)
            if distance < 13:
                object_location_color = (255, 255, 0)
                Hint1_location_x = (510, 450)
                Hint1_location_y = (535, 425)
            obj_locator = cv2.circle(rgb_img, object_location, 25, object_location_color, 3)
            # ------- --------------- -------

            # ------- Arrow -------
            robot_location = (int(pixel_x_r), int(pixel_y_r))
            arrow = cv2.arrowedLine(rgb_img, robot_location, object_location, (0, 255 ,0), 5)
            # ------- ----- -------

            # ------- robot end effector -------
            if pixel_x_r < 640 and pixel_y_r < 480 and pixel_x_r > 0 and pixel_y_r > 0:
                robot_location = robot_location
                robot_location_size = 15
                robot_location_color = (255, 0, 0)
            elif pixel_x_r > 640: # turn red if out of view
                robot_location = (625, int(pixel_y_r_xout_640))
                robot_location_size = 25
                robot_location_color = (0, 0, 255)
            elif pixel_x_r < 0:
                robot_location = (15, int(pixel_y_r_xout_0))
                robot_location_size = 25
                robot_location_color = (0, 0, 255)
            elif pixel_y_r > 480:
                robot_location = (int(pixel_x_r_yout_480), 465)
                robot_location_size = 25
                robot_location_color = (0, 0, 255)
            elif pixel_y_r < 0:
                robot_location = (int(pixel_x_r_yout_0), 15)
                robot_location_size = 25
                robot_location_color = (0, 0, 255)
            robot_locator = cv2.circle(rgb_img, robot_location, robot_location_size, robot_location_color, cv2.FILLED)
            # ------- ------------------ -------

            # ------- Collision Index -------
            height = np.interp(ee_z, [0.15, 0.4], [450, 200])
            #print(height)
            height_bar = (int(height), 425)
            if int(height) > 415:
                height_indicator_color = (0, 0, 255)
                height_text_color = (0, 0, 255)
                height_alarm = cv2.putText(rgb_img, '!! TOO CLOSE !!', (230, 385), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 2) # (465, 450)
                Hint2_location_x = (565, 450) 
                Hint2_location_y = (590, 425)
            else:
                height_indicator_color = (0, 255, 0)
                height_text_color = (0, 255, 0)
            height_indicator = cv2.rectangle(rgb_img, (200, 450), height_bar, height_indicator_color, cv2.FILLED)
            height_frame = cv2.rectangle(rgb_img, (200, 450), (450, 425), (255, 255, 255), 3)
            height_text = cv2.putText(rgb_img, 'Height', (280, 412), cv2.FONT_HERSHEY_COMPLEX, 0.8, height_text_color, 2)
            # ------- --------------- -------

            # ------- Starting Index -------
            # if mode == 0:
            #     Robot_status = cv2.putText(rgb_img, 'PAUSE', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            # if mode == 3:
            #     Robot_status = cv2.putText(rgb_img, 'SENDING HOME', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            # if mode == 5:
            #     Robot_status = cv2.putText(rgb_img, 'READY', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            # if mode == 2 and i <= 277:
            #     Robot_status = cv2.putText(rgb_img, 'START', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            #     i += 1
            # if miss == 1:
            #     Robot_status = cv2.putText(rgb_img, 'MISSED', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255), 2)
            # if mode == 17:
            #     Robot_status = cv2.putText(rgb_img, 'HOME', (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
                
            # if mode !=2:
            #     i = 0
            # if mode == 0:
            #     current_status = 'PAUSE'
            #     mode_color = (0, 255, 0)
            # if mode == 3:
            #     current_status = 'SENDING HOME'
            #     mode_color = (0, 255, 0)
            # if mode == 5:
            #     current_status = 'READY'
            #     mode_color = (0, 255, 0)
            # if mode == 2 and i <= 277:
            #     current_status = 'START'
            #     mode_color = (0, 255, 0)
            #     i += 1
            # if miss == 1:
            #     current_status = 'MISSED'
            #     mode_color = (0, 0, 255)
            # if mode == 17:
            #     current_status = 'HOME'
            #     mode_color = (0, 255, 0)
            # if mode !=2:
            #     i = 0
            # Robot_status = cv2.putText(rgb_img, current_status, (280, 35), cv2.FONT_HERSHEY_COMPLEX, 0.8, mode_color, 2)
            # ------- --------------- -------

            Dis_text = cv2.putText(rgb_img, 'Distance', (30, 412), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            Dis_update = cv2.putText(rgb_img, str(int(distance_cm)-11) + 'cm', (50, 442), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            #Dis_text_cm = cv2.putText(rgb_img, 'cm', (87, 442), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 2)
            Grasp_hint_text = cv2.putText(rgb_img, 'Grasping', (490, 412), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)
            Grasp_1 = cv2.rectangle(rgb_img, Hint1_location_x, Hint1_location_y, (0, 255, 0), cv2.FILLED)
            Grasp_2 = cv2.rectangle(rgb_img, Hint2_location_x, Hint2_location_y, (0, 255, 0), cv2.FILLED)
            Grasp_hint_1 = cv2.rectangle(rgb_img, (510, 450), (535, 425), (255, 255, 255), 3)
            Grasp_hint_2 = cv2.rectangle(rgb_img, (565, 450), (590, 425), (255, 255, 255), 3)

            # ------- enable to change size manually -------
            cv2.namedWindow('AR Interface', cv2.WINDOW_GUI_NORMAL) # or can use cv2.WINDOW_KEEPRATIO in the last parameter
            cv2.resizeWindow('AR Interface', 1280, 960)
            # cv2.imshow('AR Interface', arrow)
            cv2.imshow('AR Interface', obj_locator)
            # cv2.imshow('AR Interface', robot_locator)
            key = cv2.waitKey(1)
            # ------- ------------------------------ -------      

            # ####### create UI for ar interface #######
