#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import rospy
import serial
import time
import signal
import sys

# import message for command and feedback to/from servo
from kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3mservo_rosdriver.msg import Servo_info

# import preset functions from other python file in same directory
import Kondo_B3M_functions as Kondo_B3M

pre_target_torque = 0
id = 0
initial_process_flag = 1
MINIMUM_STEP_OF_TARGET_TORQUE = 300

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200
BATTERY_VOLTAGE_FATAL = 13800

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def set_servo_id():
    global id
    if rospy.has_param('servo_id'):
        id = rospy.get_param('servo_id')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the ID of servo. Plsease command '$rosparam set /servo_id YOUR_ID' or set it via launch file.")
    try:
        if id < 0:
            raise Exception()
    except:
        rospy.logerr("value error: servo_id")
        sys.exit(1)
    return id


def torque_control(servo_command):
    global pre_target_torque, id, initial_process_flag

    target_torque = servo_command.target_torque

    # execute only single time
    if initial_process_flag == 1:
        id = set_servo_id()
        Kondo_B3M.resetServo(id)
        Kondo_B3M.enFreeServo(id)
        Kondo_B3M.reset_encoder_total_count(id)
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Kondo_B3M.change_servocontrol_mode(id, 8)
        print("")
        rospy.logwarn("you are controlling servo ID : " + str(id) +
                      ". If you want to change the ID, abort this code and try again after execute <$ rosparam set /servo_id YOUR_ID>, or change it via launch file.")
        initial_process_flag = 0

    # ramp target torque since drastic difference of target torque may cause lock of servo
    target_torque = ramp_target_torque(target_torque, pre_target_torque)
    Kondo_B3M.control_servo_by_Torque(id, target_torque)
    publish_servo_info()
    pre_target_torque = target_torque


def publish_servo_info():
    global id, BATTERY_VOLTAGE_WARN, BATTERY_VOLTAGE_FATAL, battery_voltage_warn_flag, battery_voltage_fatal_flag
    servo_info.encoder_count = Kondo_B3M.get_encoder_total_count(id)

    voltage = Kondo_B3M.get_servo_voltage(id)
    if voltage < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag = 1
    elif voltage < BATTERY_VOLTAGE_FATAL and battery_voltage_fatal_flag == 0:
        rospy.logfatal('battery voltage is low !')
        battery_voltage_fatal_flag = 1
    servo_info.input_voltage = voltage

    servo_info.motor_velocity = Kondo_B3M.get_servo_Velocity(id)
    servo_info_pub.publish(servo_info)


def enfree_servo_after_node_ends(signal, frame):
    global id
    Kondo_B3M.enFreeServo(id)
    sys.exit(0)


def ramp_target_torque(torque_command, previous_torque_command):
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
    rospy.init_node('torque_control')

    servo_info_pub = rospy.Publisher(
        'servo_info', Servo_info, queue_size=1)
    servo_info = Servo_info()

    rospy.Subscriber('servo_command', Servo_command,
                     torque_control, queue_size=1)

    rospy.spin()
