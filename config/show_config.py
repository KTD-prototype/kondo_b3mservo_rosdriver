#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import rospy
import serial
import time
import signal
import sys
from std_msgs.msg import Int16
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info
import Kondo_B3M_functions as Kondo_B3M

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

ID = 3

position_minLimit = Kondo_B3M.read_position_minLimit()
position_maxLimit =


def initial_process():
    global id, num
    for i in range(255):
        result = Kondo_B3M.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1


if __name__ == '__main__':
    initial_process()

    rospy.Subscriber('multi_servo_command', Multi_servo_command,
                     callback_servo_command, queue_size=1)

    rospy.spin()
