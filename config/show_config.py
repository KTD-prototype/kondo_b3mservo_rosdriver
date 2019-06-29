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
current_limit = []
gains_all = []
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
        position_minLimit.append(Config.read_position_minLimit(
            id[i]))
        position_MaxLimit.append(Config.read_position_MaxLimit(id[i]))
        current_limit.append(Config.read_current_limit(id[i]))
        gains_all.append(Config.read_preset_gains(id[i]))


def show_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        print("-----------------------------------------------")
        print("CONFIGURATION OF SERVO ID : " + str(id[i]))
        print("     minimum position : " +
              str(position_minLimit[i] / 100) + "[deg]")
        print("     Maximum position : " +
              str(position_MaxLimit[i] / 100) + "[deg]")
        print("     Current limit    : " + str(current_limit[i]) + "[mA]")
        gain_for_specific_id = gains_all[i]

        for j in range(3):
            preset_gains = []
            for k in range(5):
                preset_gains.append(gain_for_specific_id[j * 5 + k])
            print("     preset gains " + str(j) + "  " + str(preset_gains))


def initial_process():
    global id, num
    for i in range(255):
        result = Config.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1


if __name__ == '__main__':
    initial_process()
    get_config()
    show_config()
