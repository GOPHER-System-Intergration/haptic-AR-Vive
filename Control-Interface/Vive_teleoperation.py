#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
####

import sys
import rospy
import numpy as np
import math
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, \
    euler_from_matrix

flagg = 0
flagg_home = 0
gripper_val = 0
vive_x = 0.2
vive_y = 0
vive_z = 0.3
vive_an_x = -90
vive_an_y = 180
vive_an_z = -90
vive_base_button_b = None
vive_base_axes_b = None
vive_stop = 0
rec_100 = 0
vive_gp = 0
vive_home = 0
vive_old_x = 0.2
vive_old_y = 0
vive_old_z = 0.3
vive_old_theta_x = -90
vive_old_theta_y = 180
vive_old_theta_z = -90
dist = 0


class ExampleFullArmMovement:
    def __init__(self):

        # self.raw_loc = np.array([0, 0, 0])

        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(
                self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(
                self.is_gripper_present))

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name,
                                                                    SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
                                                                PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # For Vive ########################
            self.vive_data = rospy.Subscriber('/Right_Hand', TransformStamped, self.callback_vive, queue_size=1)
            self.vive_b = rospy.Subscriber('/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_b, queue_size=1)
            self.pub_vel = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)
            self.z_dis = rospy.Publisher('/z_condition', Float64, queue_size=1)
            rospy.Subscriber('/Camera_distance', Float64, self.callback_camera)

            # For Vive ########################

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    # For Vive ########################
    def callback_camera(self, data):

        global dist
        dist = data.data

    def callback_vive(self, data):

        global vive_z, vive_y, vive_x, vive_an_x, vive_an_y, vive_an_z

        vive_x = data.transform.translation.x
        vive_y = data.transform.translation.y
        vive_z = data.transform.translation.z - 0.96 + 0.25
        # print("vive_x_raw", vive_x)
        # print("vive_y_raw", vive_y)
        # print("vive_z_raw", vive_z)
        rpy = data.transform.rotation
        vive_an_rpy = [rpy.x, rpy.y, rpy.z, rpy.w]
        vive_an_x, vive_an_y, vive_an_z = euler_from_quaternion(vive_an_rpy)
        # print(vive_an_z)
        # print(vive_an_x)
        # print(vive_an_y)
        # print(vive_an_z)

    def callback_vive_b(self, data):
        global vive_base_button_b, vive_base_axes_b, flagg, flagg_home, vive_stop, gripper_val, rec_100, vive_home

        vive_base_button_b = data.buttons
        vive_base_axes_b = data.axes

        # print("buttons")

        gripper_val = vive_base_axes_b[2]

        if gripper_val == 1:  # Trigger button to hold the gripper state
            rec_100 += 1
            rospy.sleep(0.5)

        if vive_base_button_b[3] == 1:  # Side button to start control
            flagg_home = 0
            flagg = 1
            rospy.sleep(0.5)
            print("started")
        if vive_base_button_b[0] == 1:  # Menu button to set starting position
            flagg_home = 1
            rospy.sleep(0.5)
            print("starting")
        '''if vive_base_button_b[2] == 1:  # Touchpad as the stop button
            if vive_home % 2 == 0 and vive_home != 0:
                vive_stop += 1
                print("pause", vive_stop)
                rospy.sleep(0.5)'''

        if vive_base_button_b[0] == 1:
            vive_home += 1
            print("home", vive_home)
            rospy.sleep(0.5)

    def example_send_cartesian_pose(self):

        global vive_z, vive_y, vive_x, vive_an_x, vive_an_y, vive_an_z

        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        # feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = 0.35
        req.input.target_pose.y = -0.1
        req.input.target_pose.z = 0.25
        req.input.target_pose.theta_x = 180 #math.degrees(vive_an_x)
        req.input.target_pose.theta_y = 0 #math.degrees(vive_an_y)
        req.input.target_pose.theta_z = 90 #math.degrees(vive_an_z)

        pose_speed = CartesianSpeed()
        pose_speed.translation = 1
        pose_speed.orientation = 50

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the starting pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return True

    # For Vive ########################

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return True

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_gripper_command(self, value):
        # Initialize the request
        # This works for the Robotiq Gripper 2F_85
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            return True

    def main(self):
        global flagg, flagg_home, dist
        global vive_x, vive_y, vive_z, vive_an_x, vive_an_y, vive_an_z
        global vive_base_button_b, vive_base_axes_b, vive_stop, rec_100, vive_home, vive_old_x, vive_old_y, vive_old_z, vive_old_theta_x, vive_old_theta_y, vive_old_theta_z
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            # *******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            # *******************************************************************************

            # *******************************************************************************
            # Move the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            # rospy.sleep(5.0)
            # *******************************************************************************

            # *******************************************************************************
            # Control
            print("starting_control")
            while 1:
                # ---Starting pose---
                while vive_home == 1:
                    print("sending_home")
                    # print('Sending to starting pose')
                    self.example_send_cartesian_pose()
                    vive_old_x = vive_x
                    vive_old_y = vive_y
                    vive_old_z = vive_z
                    vive_old_theta_x = 0  # math.degrees(vive_an_x)
                    vive_old_theta_y = 0  # math.degrees(vive_an_y)
                    vive_old_theta_z = 0  # math.degrees(vive_an_z)
                    # print(vive_old_theta_x,vive_old_theta_y,vive_old_theta_z)
                    rospy.sleep(5)
                # ---Starting pose---

                # ---Motion mapping interface---
                msg_test = TwistCommand()

                i = 0

                while flagg == 1:

                    # ---Gripper control---
                    if self.is_gripper_present:
                        if rec_100 % 2 == 1:
                            success &= self.example_send_gripper_command(1)
                        else:
                            success &= self.example_send_gripper_command(gripper_val)
                    # ---Gripper control---

                    # ---Pause---
                    if vive_home % 2 == 1 and vive_home != 1 and vive_stop % 2 == 1:
                        msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                        msg_test.twist.linear_x = 0
                        msg_test.twist.linear_y = 0
                        msg_test.twist.linear_z = 0
                        msg_test.twist.angular_x = 0
                        msg_test.twist.angular_y = 0
                        msg_test.twist.angular_z = 0
                        msg_test.duration = 0
                        self.pub_vel.publish(msg_test)
                    # ---Pause---
                    speed_value_x = None
                    speed_value_y = None
                    # ---Data process for translation---
                    if vive_stop % 2 == 0:
                        if vive_home % 3 == 0 and vive_home != 0:
                            if vive_base_button_b[2] == 1:
                                msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                                msg_test.twist.linear_x = 0
                                msg_test.twist.linear_y = 0
                                msg_test.twist.linear_z = vive_base_axes_b[1] / 10
                                msg_test.twist.angular_x = 0
                                msg_test.twist.angular_y = 0
                                msg_test.twist.angular_z = 0
                                msg_test.duration = 0
                                self.pub_vel.publish(msg_test)
                            # elif vive_home % 2 == 1 and vive_home!=1:
                            elif vive_base_button_b[2] == 0:
                                msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                                msg_test.twist.linear_x = vive_base_axes_b[1] / 10
                                msg_test.twist.linear_y = -vive_base_axes_b[0] / 10
                                msg_test.twist.linear_z = 0
                                msg_test.twist.angular_x = 0
                                msg_test.twist.angular_y = 0
                                msg_test.twist.angular_z = 0
                                msg_test.duration = 0
                                self.pub_vel.publish(msg_test)
                        if vive_home % 3 == 1 and vive_home != 1:
                            if vive_base_button_b[2] == 1:
                                msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                                msg_test.twist.linear_x = 0
                                msg_test.twist.linear_y = 0
                                msg_test.twist.linear_z = 0
                                msg_test.twist.angular_x = 0
                                msg_test.twist.angular_y = 0
                                msg_test.twist.angular_z = vive_base_axes_b[1] / 2
                                msg_test.duration = 0
                                self.pub_vel.publish(msg_test)
                            # elif vive_home % 2 == 1 and vive_home!=1:
                            elif vive_base_button_b[2] == 0:
                                msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                                msg_test.twist.linear_x = 0
                                msg_test.twist.linear_y = 0
                                msg_test.twist.linear_z = 0
                                msg_test.twist.angular_x = vive_base_axes_b[0] / 2
                                msg_test.twist.angular_y = vive_base_axes_b[1] / 2
                                msg_test.twist.angular_z = 0
                                msg_test.duration = 0
                                self.pub_vel.publish(msg_test)
                        # print("vive_x", vive_x)
                        # rint("vive_old", vive_old_x)
                        if vive_home % 3 == 2 and vive_home != 1:
                            if vive_x - vive_old_x > 0.01 or vive_x - vive_old_x < -0.01:
                                speed_x = vive_x - vive_old_x
                            else:
                                speed_x = 0

                            if vive_y - vive_old_y > 0.01 or vive_y - vive_old_y < -0.01:
                                speed_y = vive_y - vive_old_y
                            else:
                                speed_y = 0

                            if vive_z - vive_old_z > 0.01 or vive_z - vive_old_z < -0.01:
                                speed_z = vive_z - vive_old_z
                            else:
                                speed_z = 0
                            # ---Data process for translation---

                            # ---Data process for rotation---
                            # X axis - make it 0 to 180 for positive and 0 to -180 for negative
                            '''if (math.degrees(vive_an_x) > -180 and math.degrees(vive_an_x) < -160) or (math.degrees(vive_an_x) > 160 and math.degrees(vive_an_x) < 180):
                                an_speed_x = 0
                            elif math.degrees(vive_an_x) > -160 and math.degrees(vive_an_x) < 0:
                                an_speed_x = (math.degrees(vive_an_x) + 180) - vive_old_theta_x
                            else:
                                an_speed_x = (math.degrees(vive_an_x) - 180) - vive_old_theta_x
                            # Y axis - make it 0 to 90 for positive and 0 to -90 for negative
                            if math.degrees(vive_an_y) > -20 and math.degrees(vive_an_y) < 20:
                                an_speed_y = 0
                            else:
                                an_speed_y = math.degrees(vive_an_y) - vive_old_theta_y
                            # Z axis - make it 0 to 180 for positive and 0 to -180 for negative
                            if math.degrees(vive_an_z) > -20 and math.degrees(vive_an_z) < 20:
                                an_speed_z = 0
                            else:
                                an_speed_z = math.degrees(vive_an_z) - vive_old_theta_z
                            if an_speed_y > 20 or an_speed_y < -20: # To avoid rotate two axis at the same time
                                an_speed_x = 0 
                                an_speed_z = 0'''
                            # ---Data process for rotation---

                            # ---Data process for translation and orientation coupling---
                            '''if vive_y < -0.01:
                                an_speed_z = 0
                            if vive_z > 0.01:
                                an_speed_y = 0'''

                            '''if vive_an_y > 0.1 or vive_an_y < -0.1:
                                speed_z = 0
                            if vive_an_z > 0.1 or vive_an_z < -0.1:
                                speed_x = 0
                                speed_y = 0'''
                            # ---Data process for translation and orientation coupling---

                            # New rotation process
                            if vive_an_x > -0.873 and vive_an_x < 0.873:  # 50 degree
                                an_speed_x = 0
                            elif vive_an_x >= 0.873:
                                an_speed_x = 20
                            else:
                                an_speed_x = -20

                            if vive_an_y > -0.873 and vive_an_y < 0.873:
                                an_speed_y = 0
                            elif vive_an_y >= 0.873:
                                an_speed_y = 20
                            else:
                                an_speed_y = -20

                            if (vive_an_z >= 0 and vive_an_z < 1.05) or (vive_an_z <= 0 and vive_an_z > -1.05):
                                an_speed_z = 0
                            elif (vive_an_z >= 1.05):
                                an_speed_z = 20
                            else:
                                an_speed_z = -20

                            # print("Printing_Velocity________")
                            # print(vive_y,an_speed_z)

                            # ---Senting the data---

                            msg_test.reference_frame = 3  # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                            msg_test.twist.linear_x = speed_x * 5
                            msg_test.twist.linear_y = -speed_y * 3
                            msg_test.twist.linear_z = speed_z * 3

                            # ------- Condition for collision avoidance -------

                            '''if dist > 0.08:
                                self.z_dis.publish(0)
                                msg_test.twist.linear_z = speed_z * 3  # speed_z*5
                                print('here moving up')
                            elif dist < 0.08 and speed_z < 0:
                                self.z_dis.publish(1)
                                msg_test.twist.linear_z = 0
                                print('here!!')
                            else:
                                self.z_dis.publish(0)
                                msg_test.twist.linear_z = speed_z * 3
                                print('!!!!!!!!!!!!!!!!!!!')'''

                            # ------- Condition for collision avoidance -------

                            # print("speeds")
                            # print(msg_test.twist.linear_x)
                            # print(msg_test.twist.linear_y)
                            # print(msg_test.twist.linear_z)
                            # msg_test.twist.angular_x = an_speed_x*1.5
                            # msg_test.twist.angular_y = an_speed_y*1.5
                            # msg_test.twist.angular_z = an_speed_z*1.5
                            # print("Printing_Velocity________")
                            # print(an_speed_x*1.5)
                            # print(an_speed_y*1.5)
                            # print(an_speed_z*1.5)
                            # print("angular_speed", an_speed_z)
                            msg_test.twist.angular_x = an_speed_x*5
                            msg_test.twist.angular_y = an_speed_y*5
                            msg_test.twist.angular_z = an_speed_z*5
                            msg_test.duration = 0
                            self.pub_vel.publish(msg_test)
                            # ---Senting the data---

                            # ---Tracking the previous position information---
                            i = i + 1

                            if i % 10 == 0:
                                vive_old_x = vive_x
                                vive_old_y = vive_y
                                vive_old_z = vive_z
                            # ---Tracking the previous position information---

                    # ---Motion mapping interface---
                # *******************************************************************************

            # *******************************************************************************
            # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
        # *******************************************************************************


if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()
