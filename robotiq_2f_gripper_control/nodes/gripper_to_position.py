#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from time import sleep

def reset_gripper():
    pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=1000)
    
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 0
    pub.publish(command)
    #rospy.sleep(1.0)

def activate_gripper():
    pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=1000)
    
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    pub.publish(command)
    #rospy.sleep(1.0)


def gripper_to_pos(position, force, speed, hp):
	
    hp_g = True

    rospy.loginfo("I'm in the function")

    pub = rospy.Publisher('/gripper/output', outputMsg.Robotiq2FGripper_robot_output, queue_size=1000)
    
    command = outputMsg.Robotiq2FGripper_robot_output()
    hp_g = hp

    while not hp_g:
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = speed
        command.rFR  = force
        command.rPR = position

        pub.publish(command)

        #rospy.loginfo("I'm in the while + hp: %d", hp)
        #rospy.loginfo("Position: %d", position)
        #rospy.sleep(1.0)
        hp_g = True
