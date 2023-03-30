#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy, JointState
import tf.transformations
import socket
from math import pi
from kortex_driver.msg import BaseCyclic_Feedback
import random

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, \
    euler_from_matrix

RATE = 20
dist = "0"
dist_x = 0
dist_y = 0
dist_z = 0
gripper_val = 0
velocity = 0
current = 0
z = 0
gripper_value =0


# degree to radians
def d2r(i):
    return float(i) * pi / 180.0


def euler_matrix(t=[0, 0, 0]):
    return tf.transformations.euler_matrix(t[0], t[1], t[2], 'rxyz')


# def callback_vive_b(self,data):
#         global vive_base_button_b, vive_base_axes_b, flagg, flagg_home, vive_stop, gripper_val, rec_100, vive_home

#         vive_base_button_b = data.buttons
#         vive_base_axes_b = data.axes

#         #print("buttons")

#         gripper_val = vive_base_axes_b[2]


def fillin_pos(data, rot=[0, 0, 0, 0]):
    # rot is the offset of orientation for given frame
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = float(data[0])
    pose.pose.position.y = float(data[1])
    pose.pose.position.z = float(data[2])
    pose.pose.orientation.x = float(data[3])
    pose.pose.orientation.y = float(data[4])
    pose.pose.orientation.z = float(data[5])
    pose.pose.orientation.w = float(data[6])

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.child_frame_id = "test"
    msg.transform.translation.x = -float(data[0])
    msg.transform.translation.y = float(data[1])
    msg.transform.translation.z = float(data[2])
    msg.transform.rotation.x = float(data[3])
    msg.transform.rotation.y = float(data[4])
    msg.transform.rotation.z = float(data[5])
    msg.transform.rotation.w = float(data[6])

    return pose, msg


def fillin_joy(data, ctrl_name):
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ctrl_name
    msg.axes = [float(i) for i in data[0:3]]
    msg.buttons = [int(i) for i in data[3:7]]
    return msg


def callback_camera(data):

    global dist, dist_x, dist_y, dist_z
    #print("callback", data.data)
    dist = str(data.data[2])
    dist_x = data.data[0]
    dist_y = data.data[1]
    dist_z = data.data[2]


def callback_velocity(data):
    global velocity
    # print("callback", data.data)
    velocity = float(data.velocity[-1])
    # print(velocity)


def callback_z(data):
    global z
    z = data.data

def callback_joy(data):
    global gripper_value
    gripper_value = data.buttons[0]


def callback_motor(data):
    global current, gripper_position
    current = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].current_motor)
    gripper_position = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)


def vive_status_pub():
    # connect state and reconnect counter
    global dist, gripper_val, velocity, z, dist_x, dist_y, dist_z
    global current, gripper_position, gripper_value
    con_state = False
    con_cnt = 0

    hm = PoseStamped()
    pos = hm.pose.position
    [pos.x, pos.y, pos.z] = [0, 0, 0]
    # Don't know what is the /vive/twist3 or 4 is for.
    # pub_twist_r = rospy.Publisher('/vive/twist3', PoseStamped, queue_size=1)
    # pub_twist_l = rospy.Publisher('/vive/twist4', PoseStamped, queue_size=1)

    # This one is send to control the head camera
    # pub_head    = rospy.Publisher('/vive/twist5', PoseStamped, queue_size=1)
    # pub_head  = rospy.Publisher('/vive/twist5', TransformStamped, queue_size=1)
    pub_head = rospy.Publisher('/cam_base_ctrl', PoseStamped, queue_size=1)

    # Those are send to control the end effectors
    pub_pos_r = rospy.Publisher('/Right_Hand', TransformStamped, queue_size=1)
    pub_pos_l = rospy.Publisher('/Left_Hand', TransformStamped, queue_size=1)
    #pub_button = rospy.Publisher('/Button', Float64, queue_size=1)
    tracker_pub = rospy.Publisher('/Tracker', TransformStamped, queue_size=1)

    pub_key_l = rospy.Publisher('/vive/controller_LHR_FFFB7FC3/joy', Joy, queue_size=1)
    pub_key_r = rospy.Publisher('/vive/controller_LHR_FF7FBBC0/joy', Joy, queue_size=1)

    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)
    i = 0
    k = 0
    old_j = rospy.Time.now().secs
    last_k = 0
    left_vib = 0
    triggered_once = 0
    correct = 0
    trigger = random.randint(7,13)
    temp_vib = 0
    total_vib = 0
    while not rospy.is_shutdown():
        try:
            #k = 0
            j = rospy.Time.now().secs
            rospy.Subscriber('/Camera_distance', Float64MultiArray, callback_camera)
            rospy.Subscriber('/my_gen3/base_feedback/joint_state', JointState, callback_velocity)
            rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, callback_motor)
            rospy.Subscriber('/z_condition', Float64, callback_z)
            rospy.Subscriber('/vive/controller_LHR_FFFB7FC3/joy', Joy, callback_joy)
            #trigger = random.randint(7,13)
            #print(int(j)-old_j, trigger, k, left_vib)
            #print(int(j)-old_j, trigger)
            #print(gripper_value[0])
            #print(k, left_vib)
            sock.settimeout(1)
            buffer, addr = sock.recvfrom(2048)

            if not con_state:
                print
                "connected"
                con_state = True
            buffer = buffer.split(',')

            print(int(j)-old_j, trigger)
            if k == 0 and (int(j)-old_j) == trigger:
                left_vib = 1
                k+=1
                old_j = int(j)
                triggered_once += 1
                trigger = random.randint(7,13)
                left_vib_time = rospy.Time.now().secs
                temp_vib = 1
                total_vib += 1
            elif k!=0 and (int(j) - old_j) == trigger:
                left_vib = 1
                old_j = int(j)
                trigger = random.randint(7,13)
                left_vib_time = rospy.Time.now().secs
                temp_vib = 1
                total_vib += 1
            elif triggered_once == 0:
                left_vib = 0
            else:
                left_vib = 0


            # ------- Haptic support for grasping -------
            gripper_val = float(buffer[27])
            # print(z)
            if i == 0:
                old_current = current
                old_gripper = gripper_position
                i += 1
            #print(dist_z)
            if left_vib == 1 and dist_x <= 0.2 and dist_y <= 0.1 and dist_z <= 0.15:
                dist = "701"
            elif left_vib == 0 and dist_x <= 0.2 and dist_y <= 0.1 and dist_z <= 0.15:
                dist = "700"

            #print(gripper_position, abs(current - old_current), dist, (old_gripper-gripper_position))
            if left_vib == 1 and gripper_position < 60 and abs(current-old_current) > 0.1 and float(dist) == 700 and old_gripper - gripper_position <0:
                dist = "701"
            elif left_vib == 0 and gripper_position < 60 and abs(current-old_current) > 0.1 and float(dist) == 700 and old_gripper - gripper_position <0:
                dist = "700"
            
                #dist = "0"
            # ------- Haptic support for grasping -------

            # ------- Haptic support when to0 close to the table -------
            if left_vib == 1 and float(dist_z) < 0.2 and z == 1 and float(dist) != 700:
                dist = "701"
            elif left_vib == 0 and float(dist_z) < 0.2 and z == 1 and float(dist) != 700:
                dist = "700"

            if left_vib == 1 and float(dist) != 500 and float(dist) != 501 and float(dist) != 300 and float(dist) != 301:
                temp = float(dist)
                temp = 1000+temp
                dist = str(temp)

            # print(gripper_position, abs(current - old_current), dist, (old_gripper-gripper_position))
            '''if left_vib == 1 and gripper_position < 60 and abs(current - old_current) > 0.1 and float(
                    dist) < 0.15 and old_gripper - gripper_position < 0:
                dist = "301"
            elif gripper_position < 60 and abs(current - old_current) > 0.1 and float(
                    dist) < 0.15 and old_gripper - gripper_position < 0:
                dist = "300"
            # ------- Haptic support for grasping -------

            # ------- Haptic support when to0 close to the table -------
            if left_vib == 1 and float(dist) < 0.15 and z == 1:
                dist = "501"
            elif float(dist) < 0.15 and z == 1:
                dist = "500"
            if left_vib == 1 and float(dist) != 500 and float(dist) != 501 and float(dist) != 300 and float(dist) != 301:
                temp = float(dist)
                temp = 1000+temp
                dist = str(temp)'''
            # ------- Haptic support when to0 close to the table -------
            #print(str(dist))
            #dist = str(dist)
            #print(type(dist))
            #print(gripper_value)

            if gripper_value == 1:
                #if left_vib == 1:
                    #correct +=1
                button_time = rospy.Time.now().secs
                if (button_time-left_vib_time) <= 2 and temp_vib == 1:
                    correct += 1
                    temp_vib = 0
            print("dist", dist)
            print(correct, total_vib)

            sock.sendto(dist, address)
            head_pos, head_tran = fillin_pos(buffer[1:8])
            right_pos, msg_r = fillin_pos(buffer[9:16])
            left_pos, msg_l = fillin_pos(buffer[17:24])

            joy_r = fillin_joy(buffer[25:32], 'controller_LHR_FF7FBBC0')
            joy_l = fillin_joy(buffer[33:40], 'controller_LHR_FFFB7FC3')
            racker, msg_t = fillin_pos(buffer[41:48])

            #pub_button.publish(correct)
            #print(correct)
            # print("right_axes", buffer[33:36])

            # pub_twist_r.publish(right_pos)
            pub_pos_r.publish(msg_r)
            # pub_twist_l.publish(left_pos)
            pub_pos_l.publish(msg_l)
            # pub_head.publish(head_tran)
            pub_head.publish(head_pos)
            pub_key_r.publish(joy_r)
            pub_key_l.publish(joy_l)
            tracker_pub.publish(msg_t)

            old_current = current
            old_gripper = gripper_position
            rate.sleep()
        except:
            print('no server detected, reconnecting ' + str(con_cnt))
            con_state = False
            con_cnt += 1
            rate.sleep()
            sock.sendto('0', address)


if __name__ == '__main__':
    address = ("130.215.206.121", 23023)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('2', address)
    rospy.init_node('vive_status', anonymous=True)
    # rospy.sleep(3)
    try:
        vive_status_pub()
        # i=0
        # while not rospy.is_shutdown():
        #     print(i)
        #     i+=1
        #     rospy.Subscriber('/Camera_distance', Float64, callback_camera)
    except rospy.ROSInterruptException:
        pass
