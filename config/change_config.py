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
import config_function as Config

position_minLimit = []
position_MaxLimit = []
id = []
num = 0

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def change_config():
    global id, num
    Config.change_position_minLimit(3, -32000)
    Config.change_position_MaxLimit(3, 32000)
    Config.change_current_limit(3, 10000)
    Config.change_current_limit(4, 10000)

    for i in range(num):
        Config.save_RAM_to_ROM(id[i])


def read_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        position_minLimit.append(Config.read_position_minLimit(
            id[i]))
        position_MaxLimit.append(Config.read_position_MaxLimit(id[i]))


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
        result = Config.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1


if __name__ == '__main__':
    initial_process()
    change_config()
    print("")
    print("configuration changed!")
    print("")
    read_config()
    show_config()
