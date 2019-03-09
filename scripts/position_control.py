#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial, time, rospy
from kondo_b3mservo_rosdriver.msg import servo_command
import Kondo_B3M_functions as Kondo_B3M


ser = serial.Serial('/dev/ttyUSB0', 1500000)
time.sleep(0.1)

Kondo_B3M.enFreeServo(4)
Kondo_B3M.change_servocontrol_mode(4, 0)
Kondo_B3M.set_servo_trajectory_to_5Poly(4)
Kondo_B3M.set_servo_gain_to_presets(4, 0)
Kondo_B3M.change_servocontrol_mode(4, 0)

if __name__ == '__main__':
    rospy.init_node('position_control')
    rospy.Subscriber('generate_command', servo_command, position_control)
    rospy.spin()

def position_control(servo_command):
    Kondo_B3M.control_servo_by_position_without_time(4, servo_command.target_position)
