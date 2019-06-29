#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import math
import sys


# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)


# initialize (= reset) servo whose id is "ID"
def initServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x02 + 0x28 + 0x01) & 0b11111111
    enFreeServo_command = []
    enFreeServo_command += [chr(0x08), chr(0x04), chr(0x00),
                            chr(ID), chr(0x02), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(enFreeServo_command)
    # print("set servo ID:" + str(ID) + " to FREE mode")
    time.sleep(0.002)  # wait until this process done
    if ser.inWaiting() == 5:
        ret = 1
    else:
        ret = 0
    return ret


# reset servo whose id is "ID"
def resetServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x06 + 0x05 + 0x00 + ID + 0x02) & 0b11111111
    resetServo_command = []
    resetServo_command += [chr(0x06), chr(0x05),
                           chr(0x00), chr(ID), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(resetServo_command)
    time.sleep(0.1)  # wait until this process done
    print("Reset servo ID:" + str(ID))


# enfree servo whose id is "ID"
def enFreeServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x02 + 0x28 + 0x01) & 0b11111111
    enFreeServo_command = []
    enFreeServo_command += [chr(0x08), chr(0x04), chr(0x00),
                            chr(ID), chr(0x02), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(enFreeServo_command)
    time.sleep(0.1)  # wait until this process done
    print("set servo ID:" + str(ID) + " to FREE mode")


# IDが"ID"なサーボを位置制御モード、スタンバイにする関数（軌道生成：別途指定、　制御ゲイン：プリセット#0）
# mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
def change_servocontrol_mode(ID, mode):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + mode + 0x28 + 0x01) & 0b11111111
    change_servocontrol_mode_command = []
    change_servocontrol_mode_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(mode), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(change_servocontrol_mode_command)
    time.sleep(0.1)  # wait until this process done

    if mode == 0:
        set_servo_gain_to_presets(ID, 0)
        print("set servo ID:" + str(ID) +
              " to position control mode with preset gain #0")
    elif mode == 4:
        set_servo_gain_to_presets(ID, 1)
        print("set servo ID:" + str(ID) +
              " to velocity control mode with preset gain #1")
    elif mode == 8:
        set_servo_gain_to_presets(ID, 2)
        print("set servo ID:" + str(ID) +
              " to current(torque) control mode with preset gain #2")
    elif mode == 12:
        print("set servo ID:" + str(ID) + " to feed-forward control mode")


# IDが"ID"なサーボの位置制御モード時の軌道生成を5-polyモードにする関数
def set_servo_trajectory_to_5Poly(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x05 + 0x29 + 0x01) & 0b11111111
    set_servo_trajectory_to_5Poly_command = []
    set_servo_trajectory_to_5Poly_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(0x05), chr(0x29), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_trajectory_to_5Poly_command)
    time.sleep(0.1)  # wait until this process done

    print("set servo ID:" + str(ID) + " to 5-poly Trajectory")


# IDが"ID"なサーボの位置制御モード時の軌道生成をEVENモード（等速）にする関数
def set_servo_trajectory_to_EVEN(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x01 + 0x29 + 0x01) & 0b11111111
    set_servo_trajectory_to_EVEN_command = []
    set_servo_trajectory_to_EVEN_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(0x01), chr(0x29), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_trajectory_to_EVEN_command)
    time.sleep(0.1)  # wait until this process done

    print("set servo ID:" + str(ID) + " to Even Trajectroy")


# IDが"ID"なサーボの制御ゲインをプリセットのものに設定する関数
# プリセット0:位置制御用、1:速度制御用、2:トルク制御用
def set_servo_gain_to_presets(ID, PresetNumber):
    ser.reset_input_buffer()  # 返信データを読み取ってバッファから消しておく
    SUM = (0x08 + 0x04 + 0x00 + ID + PresetNumber + 0x5c + 0x01) & 0b11111111
    set_servo_gain_to_presets_command = []
    set_servo_gain_to_presets_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(PresetNumber), chr(0x5c), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_gain_to_presets_command)
    time.sleep(0.1)  # wait until this process done


# サーボの電流制限値を変更する関数
def change_current_limit(ID, current_limit_mA):
    SUM = (0x09 + 0x04 + 0x00 + ID + (current_limit_mA & 0xff) +
           (current_limit_mA >> 8) + 0x11 + 0x01) & 0b11111111
    change_current_limit_command = []
    change_current_limit_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(
        current_limit_mA & 0xff), chr(current_limit_mA >> 8), chr(0x11), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(change_current_limit_command)
    time.sleep(0.1)

    current_limit = read_current_limit(ID)
    print("set current limit of servo ID: " +
          str(ID) + " as " + str(current_limit) + "[mA]")


# サーボの電流制限値を読み取る関数
def read_current_limit(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x11 + 0x02) & 0b11111111
    read_current_limit_command = []
    read_current_limit_command += [chr(0x07), chr(0x03),
                                   chr(0x00), chr(ID), chr(0x11), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(read_current_limit_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    current_limit1 = ser.read(1)
    current_limit2 = ser.read(1)
    current_limit1 = ord(current_limit1)
    current_limit2 = ord(current_limit2)

    current_limit = (current_limit2 << 8) | current_limit1
    # print("current limit of servo ID: " + str(ID) +
    #       " is " + str(current_limit) + "[mA]")
    return current_limit


def change_position_minLimit(ID, new_limit):
    if new_limit < 0:
        new_limit = new_limit + 65536

    SUM = (0x09 + 0x04 + 0x00 + ID + (new_limit & 0xff) +
           (new_limit >> 8) + 0x05 + 0x01) & 0b11111111
    change_position_minLimit_command = []
    change_position_minLimit_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(
        new_limit & 0xff), chr(new_limit >> 8), chr(0x05), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(change_position_minLimit_command)
    time.sleep(0.1)
    # print("set current limit of servo ID: " +
    #       str(ID) + " as " + str(current_limit) + "[mA]")


def change_position_MaxLimit(ID, new_limit):
    if new_limit < 0:
        new_limit = new_limit + 65536

    SUM = (0x09 + 0x04 + 0x00 + ID + (new_limit & 0xff) +
           (new_limit >> 8) + 0x07 + 0x01) & 0b11111111
    change_position_MaxLimit_command = []
    change_position_MaxLimit_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(
        new_limit & 0xff), chr(new_limit >> 8), chr(0x07), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(change_position_MaxLimit_command)
    time.sleep(0.1)
    # print("set current limit of servo ID: " +
    #       str(ID) + " as " + str(current_limit) + "[mA]")


def read_position_minLimit(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x05 + 0x02) & 0b11111111
    read_position_minLimit_command = []
    read_position_minLimit_command += [chr(0x07), chr(0x03),
                                       chr(0x00), chr(ID), chr(0x05), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(read_position_minLimit_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    position_minLimit1 = ser.read(1)
    position_minLimit2 = ser.read(1)
    position_minLimit1 = ord(position_minLimit1)
    position_minLimit2 = ord(position_minLimit2)

    position_minLimit = (position_minLimit2 << 8) | position_minLimit1
    if position_minLimit > 32000:
        position_minLimit = position_minLimit - 65536
    # print("minimum position limit of servo ID: " + str(ID) +
    #       " is " + str(position_minLimit) + "[*0.01 deg]")
    return position_minLimit


def read_position_MaxLimit(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x07 + 0x02) & 0b11111111
    read_position_MaxLimit_command = []
    read_position_MaxLimit_command += [chr(0x07), chr(0x03),
                                       chr(0x00), chr(ID), chr(0x07), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(read_position_MaxLimit_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    position_MaxLimit1 = ser.read(1)
    position_MaxLimit2 = ser.read(1)
    position_MaxLimit1 = ord(position_MaxLimit1)
    position_MaxLimit2 = ord(position_MaxLimit2)

    position_MaxLimit = (position_MaxLimit2 << 8) | position_MaxLimit1
    if position_MaxLimit > 32000:
        position_MaxLimit = position_MaxLimit - 65536
    # print("Maximum position limit of servo ID: " + str(ID) +
    #       " is " + str(position_MaxLimit) + "[*0.01 deg]")
    return position_MaxLimit


def read_time_for_determine_that_servo_is_locked(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x14 + 0x01) & 0b11111111
    read_servo_lock_time_command = []
    read_servo_lock_time_command += [chr(0x07), chr(0x03),
                                     chr(0x00), chr(ID), chr(0x14), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(read_servo_lock_time_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 6:
            break

    Receive = ser.read(4)
    lock_time = ser.read(1)
    int_lock_time = ord(lock_time)

    print("lock time of servo ID: " + str(ID) +
          " is " + str(int_lock_time) + " [mSec]")
    return int_lock_time


def read_servo_output_to_countup_time_to_determine_that_servo_is_locked(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x15 + 0x01) & 0b11111111
    read_servo_lock_output_command = []
    read_servo_lock_output_command += [chr(0x07), chr(0x03),
                                       chr(0x00), chr(ID), chr(0x15), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(read_servo_lock_output_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 6:
            break

    Receive = ser.read(4)
    lock_output = ser.read(1)
    int_lock_output = ord(lock_output)

    print("lock output of servo ID: " + str(ID) +
          " is " + str(int_lock_output) + " [%]")
    return int_lock_output


def save_RAM_to_ROM(ID):
    SUM = (0x05 + 0x02 + 0x00 + ID) & 0b11111111
    save_RAM_to_ROM_command = []
    save_RAM_to_ROM_command += [chr(0x05),
                                chr(0x02), chr(0x00), chr(ID), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(save_RAM_to_ROM_command)
    time.sleep(0.1)
    # print("save parameters to ROM of servo ID: " + str(ID))
