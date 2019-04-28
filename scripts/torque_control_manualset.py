#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import rospy
import serial
import time
import signal
import sys
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info
import Kondo_B3M_functions as Kondo_B3M

target_torque = []
pre_target_torque = []
id = []
num = 0
initial_process_flag = 1
MINIMUM_STEP_OF_TARGET_TORQUE = 300

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200
BATTERY_VOLTAGE_FATAL = 13800
voltage = []

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def set_the_num_of_servo():
    global num
    if rospy.has_param('num_of_servo'):
        num = rospy.get_param('num_of_servo')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the number of servos. Plsease command '$rosparam set /num_of_servo THE_NUMBER_OF_SERVOS'")
        print("")
    try:
        if num < 0:
            raise Exception()
    except:
        rospy.logerr("value error: the number of servos")
        sys.exit(1)
    return num


def set_servo_id():
    global id
    if rospy.has_param('~multi_servo_id'):
        id = rospy.get_param('~multi_servo_id')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the IDs of servos. Plsease command '$rosparam set /multi_servo_id [ID1,ID2,etc]]'")
    try:
        if id < 0:
            raise Exception()
    except:
        rospy.logerr("value error: servo_id")
        sys.exit(1)
    return id


def callback_multi_torque_control(multi_servo_command):
    global num, id, initial_process_flag, target_torque, pre_target_torque, voltage

    # execute only single time
    if initial_process_flag == 1:
        num = set_the_num_of_servo()
        id = set_servo_id()

        for i in range(num):
            Kondo_B3M.resetServo(id[i])
            Kondo_B3M.enFreeServo(id[i])
            Kondo_B3M.reset_encoder_total_count(id[i])
            # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
            Kondo_B3M.change_servocontrol_mode(id[i], 8)
            pre_target_torque.append(0)
        print("")
        rospy.logwarn("you are controlling [" + str(num) + "] servos whose IDs is : " + str(id) +
                      " with TORQUE CONTROL MODE. If you want to change the number of servos or their IDs, abort this code and try again after execute <$ rosparam set /num_of_servo THE_NUMBER_OF_SERVOS> and <$ rosparam set /multi_servo_id [YOUR_ID#1, YOUR_ID#2 etc]> or change them via launch file")
        initial_process_flag = 0

    target_torque = multi_servo_command.target_torque
    target_torque = list(target_torque)

    for i in range(num):
        # damp target torque since drastic difference of target torque may cause lock of servo
        target_torque[i] = damp_target_torque(
            target_torque[i], pre_target_torque[i])
        # print(str(target_torque))
        Kondo_B3M.control_servo_by_Torque(id[i], target_torque[i])
        pre_target_torque[i] = target_torque[i]

    publish_servo_info()


def publish_servo_info():
    global id, num, battery_voltage_warn_flag, battery_voltage_fatal_flag, voltage
    multi_servo_info = Multi_servo_info()

    for i in range(num):
        multi_servo_info.encoder_count.append(
            Kondo_B3M.get_encoder_total_count(id[i]))
        multi_servo_info.input_voltage.append(
            Kondo_B3M.get_servo_voltage(id[i]))
        voltage = multi_servo_info.input_voltage

        if voltage[i] < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
            print("")
            rospy.logwarn('battery voltage is low !')
            battery_voltage_warn_flag = 1
        elif voltage[i] < BATTERY_VOLTAGE_FATAL and battery_voltage_fatal_flag == 0:
            print("")
            rospy.logfatal('battery voltage is fatally low !')

        multi_servo_info.motor_velocity.append(
            Kondo_B3M.get_servo_Velocity(id[i]))
    multi_servo_info_pub.publish(multi_servo_info)
    del multi_servo_info


def enfree_servo_after_node_ends(signal, frame):
    global id
    for i in range(num):
        Kondo_B3M.enFreeServo(id[i])
    sys.exit(0)


def damp_target_torque(torque_command, previous_torque_command):
    if abs(torque_command) > abs(previous_torque_command) + MINIMUM_STEP_OF_TARGET_TORQUE:
        if torque_command > 0:
            torque_command = previous_torque_command + MINIMUM_STEP_OF_TARGET_TORQUE
        elif torque_command < 0:
            torque_command = previous_torque_command - MINIMUM_STEP_OF_TARGET_TORQUE
    elif torque_command * previous_torque_command < 0 and abs(previous_torque_command) > MINIMUM_STEP_OF_TARGET_TORQUE:
        torque_command = 0
    return torque_command


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('multi_torque_control')
    node_name = rospy.get_name()

    multi_servo_info_pub = rospy.Publisher(
        'multi_servo_info', Multi_servo_info, queue_size=1)

    rospy.Subscriber('multi_servo_command', Multi_servo_command,
                     callback_multi_torque_control, queue_size=1)

    rospy.spin()
