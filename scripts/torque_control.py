#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import rospy
import serial
import time
import signal
import sys
from kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3mservo_rosdriver.msg import Servo_info
import Kondo_B3M_functions as Kondo_B3M

pre_target_torque = 0
id = 0
flag = 1
MINIMUM_STEP_OF_TARGET_TORQUE = 150

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def torque_control(servo_command):
    global pre_target_torque, id, flag

    id = servo_command.servo_id
    target_torque = servo_command.target_torque

    # execute only single time
    if flag == 1:
        Kondo_B3M.resetServo(id)
        Kondo_B3M.enFreeServo(id)
        Kondo_B3M.reset_encoder_total_count(id)
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Kondo_B3M.change_servocontrol_mode(id, 8)
        flag = 0

    # damp target torque since drastic difference of target torque may cause lock of servo
    target_torque = damp_target_torque(target_torque, pre_target_torque)
    # print(str(target_torque))
    Kondo_B3M.control_servo_by_Torque(id, target_torque)
    publish_servo_info()
    pre_target_torque = target_torque


def damp_target_torque(torque_command, previous_torque_command):

    if abs(torque_command) > abs(previous_torque_command):
        if torque_command > 0:
            torque_command = previous_torque_command + MINIMUM_STEP_OF_TARGET_TORQUE
        elif torque_command < 0:
            torque_command = previous_torque_command - MINIMUM_STEP_OF_TARGET_TORQUE
    elif torque_command * previous_torque_command < 0:
        torque_command = 0
    return torque_command


def publish_servo_info():
    global id
    servo_info.encoder_count = Kondo_B3M.get_encoder_total_count(id)
    servo_info.input_voltage = Kondo_B3M.get_servo_voltage(id) / 1000.0
    servo_info.motor_velocity = Kondo_B3M.get_servo_Velocity(id)
    servo_info_pub.publish(servo_info)
    # Kondo_B3M.get_mcu_temperature(4)
    # Kondo_B3M.get_servo_temperature(4)


def enfree_servo_after_node_ends(signal, frame):
    global id
    Kondo_B3M.enFreeServo(id)
    sys.exit(0)


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('torque_control')

    servo_info_pub = rospy.Publisher(
        'servo_info', Servo_info, queue_size=1)
    servo_info = Servo_info()

    rospy.Subscriber('servo_command', Servo_command,
                     torque_control, queue_size=1)

    rospy.spin()
