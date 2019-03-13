#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import rospy
import serial, time, signal, sys
from kondo_b3mservo_rosdriver.msg import servo_command
import Kondo_B3M_functions as Kondo_B3M


ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

Kondo_B3M.enFreeServo(4)
Kondo_B3M.change_servocontrol_mode(4, 0)
Kondo_B3M.set_servo_trajectory_to_5Poly(4)
Kondo_B3M.set_servo_gain_to_presets(4, 0)
Kondo_B3M.change_servocontrol_mode(4, 0)

def position_control(servo_command):
    Kondo_B3M.control_servo_by_position_without_time(4, servo_command.target_position)

def enfree_servo_after_node_ends(signal, frame):
    Kondo_B3M.enFreeServo(4)
    sys.exit(0)

signal.signal(signal.SIGINT, enfree_servo_after_node_ends)

if __name__ == '__main__':
    rospy.init_node('position_control')
    rospy.Subscriber('command', servo_command, position_control)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()
