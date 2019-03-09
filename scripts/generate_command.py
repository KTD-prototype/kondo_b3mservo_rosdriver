#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial, time, rospy
import Kondo_B3M_functions as Kondo_B3M
from sensor_msgs.msg import Joy
from kondo_b3mservo_rosdriver.msg import servo_command

global target_position, target_velocity, target_torque

ser = serial.Serial('/dev/ttyUSB0', 1500000)
time.sleep(0.1)

def generate_command(joy_msg):
    target_position = joy_msg.axes[0] * 32000 #left stick LR
    target_velocity = joy_msg.axes[2] * 32767 #right stick LR
    target_torque = joy_msg.axes[1] * 32767 #left stick FB


if __name__ == '__main__':
    rospy.init_node('generate_command')
    rospy.Subscriber('joy', Joy, generate_command)
    pub = rospy.Publisher('generate_command', servo_command, queue_size = 1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            servo_command = servo_command()
            servo_command.target_position = target_position
            servo_command.target_velocity = target_velocity
            servo_command.target_torque = target_torque
            servo_command.encoder_total_count = 0
            pub.publish(servo_command)
        except IOError:
            rospy.logerr("error")
        rate.sleep()
