#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import rospy
import Kondo_B3M_functions as Kondo_B3M
from sensor_msgs.msg import Joy
from kondo_b3mservo_rosdriver.msg import Servo_command

#global target_position, target_velocity, target_torque, pre_target_torque


ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

# pre_target_torque = 0
# MINIMUM_STEP_OF_TARGET_TORQUE = 200


def generate_command(joy_msg):
    global pre_target_torque, MINIMUM_STEP_OF_TARGET_TORQUE
    target_position = joy_msg.axes[0] * 32000  # left stick LR
    target_velocity = joy_msg.axes[3] * 32767  # right stick LR
    target_torque = joy_msg.axes[1] * 7000  # left stick FB
    # joy_msg.axes[4] : right stick FB

    servo_command = Servo_command()
    servo_command.target_position = target_position
    servo_command.target_velocity = target_velocity
    servo_command.target_torque = target_torque
    pub.publish(servo_command)
    pre_target_torque = target_torque


if __name__ == '__main__':
    rospy.init_node('generate_command')
    rospy.Subscriber('joy', Joy, generate_command, queue_size=5)
    pub = rospy.Publisher('servo_command', Servo_command, queue_size=5)
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     rate.sleep()
    rospy.spin()
