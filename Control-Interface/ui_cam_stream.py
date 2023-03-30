import threading
import cv2
import os
import numpy as np
import rospy
import tf2_ros
import tf.transformations
import math
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Pose
from kortex_driver.msg import BaseCyclic_Feedback
import json
import copy
from argparse import ArgumentParser

vive_base_button_b = (0, 0, 0, 0)
vive_base_axes_b = (0, 0, 0)


class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.t = 0
        self.width = 640
        self.height = 480
        self.camera_x = 0
        self.camera_y = 0
        self.camera_z = 0
        self.camera_x_2 = 0
        self.camera_y_2 = 0
        self.camera_z_2 = 0
        self.c_b_x = 0
        self.c_b_y = 0
        self.c_b_z = 0
        self.c_b_x_2 = 0
        self.c_b_y_2 = 0
        self.c_b_z_2 = 0
        self.pixel_x = 0
        self.pixel_y = 0
        self.pixel_x_2 = 0
        self.pixel_y_2 = 0
        self.height = 200
        self.flagg = 0
        self.vive_home = 0
        self.vive_stop = 0
        self.gaze_x = 0
        self.gaze_y = 0
        self.rec_100 = 0
        self.gripper_current = 0
        self.gripper_position = 0  # fully open
        self.vive_base_button_b = (0, 0, 0, 0)
        self.vive_base_axes_b = (0, 0, 0)
        self.rgb_img = np.zeros((480, 640, 3), np.uint8)
        self.rgb_img_1 = np.zeros((480, 640, 3), np.uint8)
        self.rgb_img_2 = np.zeros((480, 640, 3), np.uint8)
        self.circle_coord = (0, 0)
        self.pause = 0
        self.ee = TransformStamped()

        # self.cap_rs1 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        # self.cap_rs2 = cv2.VideoCapture(
        #    'v4l2src device=/dev/video10 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink',
        #    cv2.CAP_GSTREAMER)

        self.out_send = cv2.VideoWriter(
            'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=130.215.181.94 port=2332 sync=false',
            cv2.CAP_GSTREAMER, 0, 25, (640, 480))

        self.running = True

        rospy.init_node('cam_test')
        # From Vive ########################
        self.vive_b = rospy.Subscriber('/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_b, queue_size=1)
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        rospy.Subscriber('/ee_loc', TransformStamped, self.callback_ee)
        rospy.Subscriber('/aruco_simple/pose2', Pose, self.callback_camera)
        rospy.Subscriber('/aruco_simple/pose', Pose, self.callback_camera_marker2)
        self.eye_x = rospy.Subscriber('/Gaze_X', Float64, self.callback_gaze_x, queue_size=1)
        self.eye_y = rospy.Subscriber('/Gaze_Y', Float64, self.callback_gaze_y, queue_size=1)
        self.main_image = rospy.Publisher('/main_detection', String, queue_size=1)
        self.ar_image = rospy.Publisher('/ar_detection', String, queue_size=1)
        rospy.Subscriber('/my_left_arm/base_feedback', BaseCyclic_Feedback, self.callback_motor)
        self.ee_loc = rospy.Publisher('/EE', Float64MultiArray, queue_size=1)
        self.button_press = rospy.Publisher('/button_press', String, queue_size=1)
        self.target_pub = rospy.Publisher('/target_indicator', Float64, queue_size=1)
        # From Vive ########################

    def circle_gaze(self, ee_loc, gaze):

        value = abs(np.linalg.norm(np.asarray(gaze) - np.asarray(ee_loc)))

        if value <= 50:

            gaze_value = 1

        else:

            gaze_value = 0

        return gaze_value
    
    def callback_ee(self, data):

        # print(data)
        self.ee = data

    def callback_motor(self, data):
        # global gripper_current, gripper_position
        self.gripper_current = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].current_motor)
        self.gripper_position = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)


    def area_gaze(self, bottom_left, top_right, gaze):

        pip_presence = 0

        if gaze[0] <= top_right[0] and gaze[0] >= bottom_left[0]:
            if gaze[1] >= top_right[1] and gaze[1] <= bottom_left[1]:
                pip_presence = 1

        return  pip_presence

    def aruco_gaze_main(self, gaze, marker):

        if abs(marker[0] - gaze[0]) <= 50 and abs(marker[1] - gaze[1]) <= 50:
            obj_presence = 1
        else:
            obj_presence = 0

        return obj_presence

    def aruco_gaze_pip(self, gaze, marker):

        if abs(marker[0] - gaze[0]) <= 10 and abs(marker[1] - gaze[1]) <= 10:
            obj_presence = 1
        else:
            obj_presence = 0

        return obj_presence

    def coord_correction(self, bottom_left):

        if bottom_left[0] < 0:
            bottom_left[0] = 15
        if bottom_left[0] > 640:
            bottom_left[0] = 625
        if bottom_left[1] < 0:
            bottom_left[1] = 15
        if bottom_left[1] >480:
            bottom_left[1] = 465

        return bottom_left


    def aruco_detect(self, x, y, z):

        try:
            pixel_x = (611.055419921875 * x / z) + 316.6255187988281
            pixel_y = (611.2431030273438 * y / z) + 229.1320343017578
        except:
            return 0, 0

        return pixel_x, pixel_y


    def callback_gaze_x(self, data):

        self.gaze_x = data.data
        # print(self.gaze_x)

    def callback_gaze_y(self, data):

        self.gaze_y = data.data
        # print(self.gaze_y)

    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def callback_vive_b(self, data):

        self.vive_base_button_b = data.buttons  # vive_base_button_b[2] will be 1 if press trackpad or side button ...
        self.vive_base_axes_b = data.axes

        gripper_val = self.vive_base_axes_b[2]

        if gripper_val == 1:  # Trigger button to hold the gripper state
            self.rec_100 += 1
            rospy.sleep(0.5)

        if self.vive_base_button_b[2] == 1:  # Side button to start control
            self.flagg = 1
            self.pause += 1
            rospy.sleep(0.5)
            # print("started")

        if self.vive_base_button_b[0] == 1:
            self.vive_home += 1
            # print("home", vive_home)
            rospy.sleep(0.5)

        if self.vive_base_button_b[2] == 1 and self.vive_base_axes_b[0] == 0:  # Side button as the stop button
            # if vive_home % 2 == 0 and vive_home != 0:
            self.vive_stop += 1
            # print("pause", self.vive_stop)
            rospy.sleep(0.5)

    def callback_camera(self, data):
        # global camera_x, camera_y, camera_z
        # global pixel_x, pixel_y
        # global c_b_x, c_b_y, c_b_z

        self.camera_x = data.position.x
        self.camera_y = data.position.y
        self.camera_z = data.position.z

        self.c_b_x = 0.5 * self.camera_z - 0.866 * self.camera_y - 0.1
        self.c_b_y = -self.camera_x + 0.35
        self.c_b_z = -0.866 * self.camera_z - 0.5 * self.camera_y + 0.85

        # ------- transfer from webcam coordinate to image plane -------
        self.pixel_x = (
                               611.055419921875 * self.camera_x / self.camera_z) + 316.6255187988281  # get it by $rostopic echo /camera/color/camera_info
        self.pixel_y = (
                               611.2431030273438 * self.camera_y / self.camera_z) + 229.1320343017578  # get it by $rostopic echo /camera/color/camera_info
        # ------- ---------------------------------------------- -------

    def callback_camera_marker2(self, data):
        # global camera_x_2, camera_y_2, camera_z_2
        # global pixel_x_2, pixel_y_2
        # global c_b_x_2, c_b_y_2, c_b_z_2

        self.camera_x_2 = data.position.x
        self.camera_y_2 = data.position.y
        self.camera_z_2 = data.position.z

        self.c_b_x_2 = 0.5 * self.camera_z_2 - 0.866 * self.camera_y_2 - 0.1
        self.c_b_y_2 = -self.camera_x_2 + 0.35
        self.c_b_z_2 = -0.866 * self.camera_z_2 - 0.5 * self.camera_y_2 + 0.85

        # ------- transfer from webcam coordinate to image plane -------
        self.pixel_x_2 = (
                                 611.055419921875 * self.camera_x_2 / self.camera_z_2) + 316.6255187988281  # get it by $rostopic echo /camera/color/camera_info
        self.pixel_y_2 = (
                                 611.2431030273438 * self.camera_y_2 / self.camera_z_2) + 229.1320343017578  # get it by $rostopic echo /camera/color/camera_info
        # ------- ---------------------------------------------- -------

    def run(self):
        # global vive_base_button_b, vive_base_axes_b
        # print(vive_base_button_b[2])

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.sleep(1)

        flag = 0
        zoom_in = 0
        zoom_out = 0
        shift_x = 0
        shift_y = 0
        rate = rospy.Rate(30)
        
        ee = Float64MultiArray()
        
        # ------- video record ------- #
        # parser = ArgumentParser()
        # parser.add_argument('filename', metavar='N', type=str, nargs='+',
        #             help='an integer for the accumulator')
        # args = parser.parse_args()
        # name_of_file = args.filename[-1]
        
        # video = cv2.VideoWriter(name_of_file + 'video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 25, (640, 480))
        # ------- video record ------- #
        
        pause = 0

        while not rospy.is_shutdown():

            # read the video stream from second realsense camera
            # ret_rs2, frame_rs2 = self.cap_rs2.read()
            
            # frame_rs1 = cv2.resize(self.rgb_img, (640, 480), interpolation = cv2.INTER_AREA)
            frame_rs1 = self.rgb_img
            main_img = copy.deepcopy(frame_rs1)
            obj1_main_detect = 0

            # get the robot end effector w.r.t. image plane: (pixel_x_r, pixel_y_r)
            # robot_ee = tfBuffer.lookup_transform('base_link', 'robotiq_arg2f_base_link', rospy.Time())
            # # ------- transfer from robot base frame to webcam coordinate -------
            ee_x = self.ee.transform.translation.x
            ee_y = self.ee.transform.translation.y
            ee_z = self.ee.transform.translation.z
            ee.data = [ee_x, ee_y, ee_z]
            ee_x_new = round(ee_x,2)
            ee_y_new = round(ee_y,2)
            ee_z_new = round(ee_z,2)
            
            # ------- Display Robot Status ------- #
            if self.vive_home == 0:
                status_text = 'WAITING'
            elif self.vive_home == 1:
                status_text = 'HOME'
            elif self.vive_home % 4 == 2 and self.vive_home != 1:
                status_text = 'READY'

            if self.flagg == 1:
                status_text = 'TELEOP'
            if self.vive_stop > 0 and self.vive_stop % 2 == 0:
                pause += 1
                status_text = 'PAUSE'

            cv2.putText(frame_rs1, status_text, (500, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame_rs1, 'SEQUENCE: ', (290, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)            
            # cv2.putText(frame_rs1, 'G-', (470, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 110, 0), 3, cv2.LINE_AA)
            # cv2.putText(frame_rs1, 'Y-', (520, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame_rs1, 'R', (570, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 139), 3, cv2.LINE_AA)
            # ------- Display Robot Status ------- #

            # ------- Convert Gaze Data ------- #
            if 1680 >= self.gaze_x >= 240:
                cam_x = (int(self.gaze_x) - 240) / 2.25
                cam_y = 480 - (int(self.gaze_y) / 2.25)
                gaze_pixel = (int(cam_x), int(cam_y))
            else:
                gaze_pixel = (0, 0)
            # ------- Convert Gaze Data ------- #

            # ------- camera control (transparent) ------- #
            # UP
            # Initialize blank mask image of same dimensions for drawing the shapes
            # shapes_up = np.zeros_like(frame_rs1, np.uint8)

            # # Draw shapes
            # cam_color = (255,255,255)
            # cv2.rectangle(shapes_up, (260, 72), (380, 2), cam_color, 2)
            # pts_up = [(270,60),(370,60),(320,10)]
            # cv2.fillPoly(shapes_up, np.array([pts_up]), cam_color)

            # # DOWN
            # # Initialize blank mask image of same dimensions for drawing the shapes
            # shapes_down = np.zeros_like(frame_rs1, np.uint8)

            # # Draw shapes
            # cam_color = (255,255,255)
            # cv2.rectangle(shapes_down, (260, 472), (380, 402), cam_color, 2)
            # pts_down = [(270,410),(370,410),(320,460)]
            # cv2.fillPoly(shapes_down, np.array([pts_down]), cam_color)

            # # RIGHT
            # # Initialize blank mask image of same dimensions for drawing the shapes
            # shapes_right = np.zeros_like(frame_rs1, np.uint8)

            # # Draw shapes
            # cam_color = (255,255,255)
            # cv2.rectangle(shapes_right, (568, 300), (638, 180), cam_color, 2)
            # pts_right = [(578,190),(578,290),(630,240)]
            # cv2.fillPoly(shapes_right, np.array([pts_right]), cam_color)

            # # LEFT
            # # Initialize blank mask image of same dimensions for drawing the shapes
            # shapes_left = np.zeros_like(frame_rs1, np.uint8)

            # # Draw shapes
            # cam_color = (255,255,255)
            # cv2.rectangle(shapes_left, (2, 300), (72, 180), cam_color, 2)
            # pts_left = [(62,190),(62,290),(12,240)]
            # cv2.fillPoly(shapes_left, np.array([pts_left]), cam_color)

            # # Generate output by blending image with shapes image, using the shapes
            # # images also as mask to limit the blending to those parts
            # out = frame_rs1.copy()
            # if 380 >= gaze_pixel[0] >=260 and 72 >=gaze_pixel[1]>= 2:
            #     alpha_up = 0
            # else:   
            #     alpha_up = 0.87     
            # if 380 >= gaze_pixel[0] >=260 and 472 >=gaze_pixel[1]>= 402:
            #     alpha_down = 0
            # else:   
            #     alpha_down = 0.87
            # if 638 >= gaze_pixel[0] >=568 and 300 >=gaze_pixel[1]>= 180:
            #     alpha_right = 0
            # else:   
            #     alpha_right = 0.87
            # if 72 >= gaze_pixel[0] >=2 and 300 >=gaze_pixel[1]>= 180:
            #     alpha_left = 0
            # else:   
            #     alpha_left = 0.87
            # mask_up = shapes_up.astype(bool)
            # mask_down = shapes_down.astype(bool)
            # mask_right = shapes_right.astype(bool)
            # mask_left = shapes_left.astype(bool)
            # out[mask_up] = cv2.addWeighted(frame_rs1, alpha_up, shapes_up, 1 - alpha_up, 0)[mask_up] 
            # out[mask_down] = cv2.addWeighted(frame_rs1, alpha_down, shapes_down, 1 - alpha_down, 0)[mask_down] 
            # out[mask_right] = cv2.addWeighted(frame_rs1, alpha_right, shapes_right, 1 - alpha_right, 0)[mask_right]  
            # out[mask_left] = cv2.addWeighted(frame_rs1, alpha_left, shapes_left, 1 - alpha_left, 0)[mask_left]  
            # ------- camera control (transparent) ------- #

            # cv2.circle(out, gaze_pixel, 15, (255, 255, 255), 3)

            # ##################### target indicator ##################### #
            object1_pixel = (515, 425)#(295, 375)
            object2_pixel = (410, 335)#(515, 425)
            object3_pixel = (295, 375)#(410, 335)
            box1_pixel = (445, 190)#(570, 290)
            box2_pixel = (570, 290)#(325, 220)
            box3_pixel = (325, 220)#(445, 190)
            dis_obj1 = int(math.hypot(gaze_pixel[0] - object1_pixel[0], gaze_pixel[1] - object1_pixel[1]))
            dis_obj2 = int(math.hypot(gaze_pixel[0] - object2_pixel[0], gaze_pixel[1] - object2_pixel[1]))
            dis_obj3 = int(math.hypot(gaze_pixel[0] - object3_pixel[0], gaze_pixel[1] - object3_pixel[1])) 
            dis_box1 = int(math.hypot(gaze_pixel[0] - box1_pixel[0], gaze_pixel[1] - box1_pixel[1])) 
            dis_box2 = int(math.hypot(gaze_pixel[0] - box2_pixel[0], gaze_pixel[1] - box2_pixel[1]))  
            dis_box3 = int(math.hypot(gaze_pixel[0] - box3_pixel[0], gaze_pixel[1] - box3_pixel[1]))
            # print(dis_obj1, dis_obj2, dis_obj3, dis_box1, dis_box2, dis_box3)

            object1_robot_frame = (0.11, 0.21, 0.15)#(0.21, 0.48, 0.15)
            object2_robot_frame = (0.24, 0.32, 0.15)#(0.15, 0.21, 0.15)
            object3_robot_frame = (0.18, 0.47, 0.15)#(0.27, 0.33, 0.15)
            box1_robot_frame = (0.47, 0.23, 0.22)#(0.34, 0.1, 0.22)
            box2_robot_frame = (0.28, 0.1, 0.22)#(0.45, 0.43, 0.22)
            box3_robot_frame = (0.42, 0.41, 0.22)#(0.52, 0.25, 0.22)
            dis_obj1_ee = int(math.sqrt((ee_x_new - object1_robot_frame[0]) ** 2 + (ee_y_new - object1_robot_frame[1]) ** 2 + (ee_z_new - object1_robot_frame[2]) ** 2)*100)
            dis_obj2_ee = int(math.sqrt((ee_x_new - object2_robot_frame[0]) ** 2 + (ee_y_new - object2_robot_frame[1]) ** 2 + (ee_z_new - object2_robot_frame[2]) ** 2)*100)
            dis_obj3_ee = int(math.sqrt((ee_x_new - object3_robot_frame[0]) ** 2 + (ee_y_new - object3_robot_frame[1]) ** 2 + (ee_z_new - object3_robot_frame[2]) ** 2)*100)
            dis_box1_ee = int(math.sqrt((ee_x_new - box1_robot_frame[0]) ** 2 + (ee_y_new - box1_robot_frame[1]) ** 2 + (ee_z_new - box1_robot_frame[2]) ** 2)*100)
            dis_box2_ee = int(math.sqrt((ee_x_new - box2_robot_frame[0]) ** 2 + (ee_y_new - box2_robot_frame[1]) ** 2 + (ee_z_new - box2_robot_frame[2]) ** 2)*100)
            dis_box3_ee = int(math.sqrt((ee_x_new - box3_robot_frame[0]) ** 2 + (ee_y_new - box3_robot_frame[1]) ** 2 + (ee_z_new - box3_robot_frame[2]) ** 2)*100)
            # print(dis_obj1_ee, dis_obj2_ee, dis_obj3_ee, dis_box1_ee, dis_box2_ee, dis_box3_ee)
            # print(ee_x_new, ee_y_new, ee_z_new)

            # ------- showing object locations -------#
            # cv2.circle(frame_rs1, object1_pixel, 15, (255, 255, 255), cv2.FILLED)
            # cv2.circle(frame_rs1, object2_pixel, 15, (255, 255, 255), cv2.FILLED)
            # cv2.circle(frame_rs1, object3_pixel, 15, (255, 255, 255), cv2.FILLED)
            # cv2.circle(frame_rs1, box1_pixel, 15, (255, 255, 255), cv2.FILLED)
            # cv2.circle(frame_rs1, box2_pixel, 15, (255, 255, 255), cv2.FILLED)
            # cv2.circle(frame_rs1, box3_pixel, 15, (255, 255, 255), cv2.FILLED)  
            # ------- showing object locations -------#

            # ------- distance only #+ task state ------- #
            if dis_obj1_ee <= 10 and self.gripper_position < 55:
                # cv2.rectangle(frame_rs1, (object1_pixel[0]-40, object1_pixel[1]+40), (object1_pixel[0]+40, object1_pixel[1]-40), (255, 255, 255), 2)
                target = 5
            
            elif dis_obj2_ee <= 10 and self.gripper_position < 55:
                # cv2.rectangle(frame_rs1, (object2_pixel[0]-40, object2_pixel[1]+40), (object2_pixel[0]+40, object2_pixel[1]-40), (255, 255, 255), 2)
                target = 1

            elif dis_obj3_ee <= 10 and self.gripper_position < 55:
                # cv2.rectangle(frame_rs1, (object3_pixel[0]-40, object3_pixel[1]+40), (object3_pixel[0]+40, object3_pixel[1]-40), (255, 255, 255), 2)
                target = 3

            elif dis_box1_ee <= 10 and self.gripper_position >= 55:
                # cv2.rectangle(frame_rs1, (box1_pixel[0]-40, box1_pixel[1]+40), (box1_pixel[0]+40, box1_pixel[1]-40), (255, 255, 255), 2)
                target = 6

            elif dis_box2_ee <= 10 and self.gripper_position >= 55:
                # cv2.rectangle(frame_rs1, (box2_pixel[0]-40, box2_pixel[1]+40), (box2_pixel[0]+40, box2_pixel[1]-40), (255, 255, 255), 2)
                target = 4

            elif dis_box3_ee <= 10 and self.gripper_position >= 55:
                # cv2.rectangle(frame_rs1, (box3_pixel[0]-40, box3_pixel[1]+40), (box3_pixel[0]+40, box3_pixel[1]-40), (255, 255, 255), 2)
                target = 2

            else:
                target = 0
            # print(target)
            # print(dis_obj1_ee, dis_obj2_ee, dis_obj3_ee)
            # print(dis_box1_ee, dis_box2_ee, dis_box3_ee)

            self.target_pub.publish(target)
            #------- gaze + distance + task state ------- #

            # ------- Secondary View ------- #
            #             out_w_sec = frame_rs1.copy()
            #             frame_zoom = frame_rs2[0+72:480-72, 0+96:640-96]
            #             resized_frame_zoom = cv2.resize(frame_zoom, (640, 480), interpolation=cv2.INTER_AREA)
            #             resized_frame = cv2.resize(resized_frame_zoom, (280, 210), interpolation=cv2.INTER_AREA)
            #             out_w_sec[2:212, 2:282] = resized_frame
            #             cv2.rectangle(out_w_sec, (2, 212), (282, 2), (255, 255, 255), 2)
            # cv2.putText(frame_rs1, 'LEFT', (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3,
            #             cv2.LINE_AA)  # orange (30, 105, 210)
            # ------- Secondary View ------- #

            

            if target == 0:
                self.out_send.write(frame_rs1)
            else:
                self.out_send.write(out_w_sec)
            rate.sleep()

            cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Viewpoint', 1280, 960)
            # target = np.random.randint(0,7)
            # cv2.imshow('Viewpoint', frame_rs1)
            if target == 0:
                cv2.imshow('Viewpoint', frame_rs1)
                # video.write(main_img) # video record
            else:
                cv2.imshow('Viewpoint', out_w_sec) # showing gaze location
                # video.write(out_frame) # video record

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            # elif key & 0xFF == ord('s'):

        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        self.out_send.release()
        cv2.destroyAllWindows()
        # threading.Thread.exit()


if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
