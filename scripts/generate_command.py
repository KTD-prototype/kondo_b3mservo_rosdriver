#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial, time, rospy
import Kondo_B3M_functions as Kondo_B3M
from sensor_msgs.msg import Joy
from kondo_b3mservo_rosdriver.msg import servo_command

#global target_position, target_velocity, target_torque, pre_target_torque


ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

def generate_command(joy_msg):
    target_position = joy_msg.axes[0] * 32000 #left stick LR
    target_velocity = joy_msg.axes[3] * 32767 #right stick LR
    target_torque = joy_msg.axes[1] * 7000 #left stick FB
    #joy_msg.axes[4] : right stick FB

    command = servo_command()
    command.target_position = target_position
    command.target_velocity = target_velocity
    command.target_torque = target_torque
    command.encoder_total_count = 0
    pub.publish(command)


if __name__ == '__main__':
    rospy.init_node('generate_command')
    rospy.Subscriber('joy', Joy, generate_command, queue_size = 5)
    pub = rospy.Publisher('command', servo_command, queue_size = 5)
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     rate.sleep()
    rospy.spin()
