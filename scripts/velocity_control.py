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

id = 0
flag = 1

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def callback_velocity_control(servo_command):
    global id, flag

    id = servo_command.servo_id
    target_velocity = servo_command.target_velocity

    if flag == 1:
        Kondo_B3M.resetServo(id)
        Kondo_B3M.enFreeServo(id)
        Kondo_B3M.reset_encoder_total_count(id)
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Kondo_B3M.change_servocontrol_mode(id, 4)
        flag = 0

    Kondo_B3M.control_servo_by_Velocity(id, target_velocity)
    publish_servo_info()


def publish_servo_info():
    global id
    servo_info.encoder_count = Kondo_B3M.get_encoder_total_count(id)
    servo_info.input_voltage = Kondo_B3M.get_servo_voltage(id)
    servo_info.motor_velocity = Kondo_B3M.get_servo_Velocity(id)
    servo_info_pub.publish(servo_info)


def enfree_servo_after_node_ends(signal, frame):
    Kondo_B3M.enFreeServo(4)
    sys.exit(0)


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)

if __name__ == '__main__':
    rospy.init_node('velocity_control')

    servo_info_pub = rospy.Publisher(
        'servo_info', Servo_info, queue_size=1)
    servo_info = Servo_info()

    rospy.Subscriber('servo_command', Servo_command,
                     callback_velocity_control, queue_size=1)

    rospy.spin()
