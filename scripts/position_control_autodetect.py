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

target_torque = []
pre_target_torque = []
id = []
num = 0
initial_process_flag = 1
found_servo_flag = 1
MINIMUM_STEP_OF_TARGET_TORQUE = 300

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200
BATTERY_VOLTAGE_FATAL = 13800
voltage = []

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def initial_process():
    global id, num, initial_process_flag, the_number_of_servo_pub
    if initial_process_flag == 1:
        for i in range(255):
            # Kondo_B3M.resetServo(i)
            result = Kondo_B3M.enFreeServo(i)
            # print(result)
            if result == 1:
                id.append(i)
                num = num + 1

        for j in range(num):
            Kondo_B3M.resetServo(id[j])
            Kondo_B3M.enFreeServo(id[j])
            Kondo_B3M.reset_encoder_total_count(id[j])
            # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
            Kondo_B3M.change_servocontrol_mode(id[j], 0)
        print("")
        rospy.logwarn("you are controlling [" + str(num) + "] servos whose IDs is : " + str(id) +
                      " at POSITION CONTROL MODE, which are automatically detected.")

        initial_process_flag = 0
        the_number_of_servo_pub.publish(num)
    else:
        pass


def callback_multi_position_control(multi_servo_command):
    global num, id, initial_process_flag, target_torque, pre_target_torque, voltage

    target_position = multi_servo_command.target_position
    target_position = list(target_position)
    # print(num)

    for i in range(num):
        Kondo_B3M.control_servo_by_position_without_time(
            id[i], target_position[i])

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


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('multi_torque_control')
    multi_servo_info_pub = rospy.Publisher(
        'multi_servo_info', Multi_servo_info, queue_size=1)

    the_number_of_servo_pub = rospy.Publisher(
        'the_number_of_servo', Int16, queue_size=1, latch=True)

    initial_process()

    rospy.Subscriber('multi_servo_command', Multi_servo_command,
                     callback_multi_position_control, queue_size=1)

    rospy.spin()
