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
sys.path.append('../scripts')
import Kondo_B3M_functions as Kondo_B3M

position_minLimit = []
position_MaxLimit = []
id = []
num = 0

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def get_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        position_minLimit.append(Kondo_B3M.read_position_minLimit(
            id[i]))
        position_MaxLimit.append(Kondo_B3M.read_position_MaxLimit(id[i]))


def show_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        print("-----------------------------------------------")
        print("CONFIGURATION OF SERVO ID : " + str(id[i]))
        print("     minimum position : " + str(position_minLimit[i]))
        print("     Maximum position : " + str(position_MaxLimit[i]))


def initial_process():
    global id, num
    for i in range(255):
        result = Kondo_B3M.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1


if __name__ == '__main__':
    initial_process()
    get_config()
    show_config()
