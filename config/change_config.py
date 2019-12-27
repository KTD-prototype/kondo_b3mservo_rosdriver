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

    # 任意のIDのサーボに対し任意の値で設定変更が可能。以下は例。

    # Config.change_position_minLimit(3, -32000)
    # Config.change_position_MaxLimit(3, 32000)
    # Config.change_current_limit(3, 10000)
    # Config.change_current_limit(4, 10000)

    Config.change_preset_gain(3, 0x6e, 20000)  # P gain for vel
    Config.change_preset_gain(3, 0x72, 200)  # D gain for vel
    Config.change_preset_gain(3, 0x76, 8000)  # I gain for vel
    Config.change_preset_gain(3, 0x7a, 0)  # stat friction for vel
    Config.change_preset_gain(3, 0x7c, 0)  # dyna friction for vel

    # Config.change_preset_gain(3, 0x7e, 5)  # P gain for tor
    # Config.change_preset_gain(3, 0x82, 0)  # D gain for tor
    # Config.change_preset_gain(3, 0x86, 70)  # I gain for tor
    # Config.change_preset_gain(3, 0x8a, 0)  # stat friction for tor
    # Config.change_preset_gain(3, 0x8c, 0)  # dyna friction for tor

    for i in range(num):
        Config.save_RAM_to_ROM(id[i])


#　変更したパラメータを確認するため、読み出し
def read_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        # 何か読み出したいなら、以下のpassはコメントアウト
        pass

        # 任意のパラメータを確認のため呼び出し
        # position_minLimit.append(Config.read_position_minLimit(
        #     id[i]))
        # position_MaxLimit.append(Config.read_position_MaxLimit(id[i]))


# 変更したパラメータを確認するため、読みだしたパラメータを表示
def show_config():
    global id, num
    global position_minLimit, position_MaxLimit
    for i in range(num):
        # print("-----------------------------------------------")
        # print("CONFIGURATION OF SERVO ID : " + str(id[i]))
        # print("     minimum position : " + str(position_minLimit[i]))
        # print("     Maximum position : " + str(position_MaxLimit[i]))
        pass


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
