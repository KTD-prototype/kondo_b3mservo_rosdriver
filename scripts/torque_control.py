#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import rospy
import serial, time, signal, sys
from kondo_b3mservo_rosdriver.msg import servo_command
import Kondo_B3M_functions as Kondo_B3M


ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

pre_target_torque = 0
MINIMUM_STEP_OF_TARGET_TORQUE = 225

Kondo_B3M.resetServo(4)
Kondo_B3M.enFreeServo(4)
Kondo_B3M.reset_encoder_total_count(4)
Kondo_B3M.change_servocontrol_mode(4, 8) #mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL

def torque_control(servo_command):
    global pre_target_torque
    target_torque = servo_command.target_torque

    #damp target torque since drastic difference of target torque may cause lock of servo
    target_torque = damp_target_torque(target_torque, pre_target_torque)

    Kondo_B3M.control_servo_by_Torque(4, target_torque)
    Kondo_B3M.get_encoder_total_count(4)
    pre_target_torque = target_torque


def damp_target_torque(torque_command, previous_torque_command):
    if torque_command - previous_torque_command < -1 * MINIMUM_STEP_OF_TARGET_TORQUE:
        torque_command = previous_torque_command - MINIMUM_STEP_OF_TARGET_TORQUE
    elif torque_command - previous_torque_command > MINIMUM_STEP_OF_TARGET_TORQUE:
        torque_command = previous_torque_command + MINIMUM_STEP_OF_TARGET_TORQUE
    return torque_command


def enfree_servo_after_node_ends(signal, frame):
    Kondo_B3M.enFreeServo(4)
    sys.exit(0)

signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('torque_control')
    rospy.Subscriber('command', servo_command, torque_control, queue_size = 1)
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     rate.sleep()
    rospy.spin()
