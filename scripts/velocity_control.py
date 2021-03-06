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
import drive_function as Drive

target_velocity = []
id = []
merged_command = []
num = 0

BATTERY_VOLTAGE_WARN = 11200
BATTERY_VOLTAGE_FATAL = 10700
voltage = []

voltage_monitor_flag = 0
servo_reset_flag = 0
servo_drive_flag = 1

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def initial_process():
    global id, num
    global target_velocity, voltage

    for i in range(255):
        result = Drive.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1

    initialize_servo_for_velocity_control()
    for j in range(num):
        target_velocity.append(0)
        voltage.append(12000)

    rospy.logwarn("you are controlling [" + str(num) + "] servos whose IDs is : " + str(id) +
                  " at VELOCITY CONTROL MODE")

    target_velocity = list(target_velocity)
    the_number_of_servo_pub.publish(num)


def callback_servo_command(multi_servo_command):
    global num, target_velocity, id, merged_command
    global servo_reset_flag, servo_drive_flag

    # actual locomotions are only conducted when the flag is active
    if servo_drive_flag == 1:
        target_velocity = multi_servo_command.target_velocity
        target_velocity = list(target_velocity)
        for i in range(num):
            merged_command.append(id[i])
        for j in range(num):
            merged_command.append(target_velocity[j])
        Drive.control_servo_by_Velocity_multicast(merged_command)
        publish_servo_info()
        merged_command = []

    # if flag for resetting servos is active
    if servo_reset_flag == 1:
        initialize_servo_for_velocity_control()
        rospy.logwarn('complete resetting servos!')
        servo_reset_flag = 0  # deactivate flag for resetting servos

        # deactivate flag for servo locomoting until it's triggered to restart
        servo_drive_flag = 0


def publish_servo_info():
    global id, num, voltage
    global servo_drive_flag, voltage_monitor_flag

    # deactivate servo_drive_flag to prevent from sending velocity command to the servos before completing this process
    servo_drive_flag = 0

    multi_servo_info = Multi_servo_info()

    # Don't have to monitor voltage at every loop, so get sparsed at a time per a certain loops
    # monitor per 100 cycles
    if voltage_monitor_flag == 100:
        voltage_monitor_flag = 0
        for j in range(num):
            voltage[j] = Drive.get_servo_voltage(id[j])
            if voltage[j] < BATTERY_VOLTAGE_WARN:
                print("")
                rospy.logwarn('battery voltage is low !')
            elif voltage[j] < BATTERY_VOLTAGE_FATAL:
                print("")
                rospy.logfatal('battery voltage is fatally low !')
    voltage_monitor_flag = voltage_monitor_flag + 1
    voltage = list(voltage)

    for i in range(num):
        multi_servo_info.encoder_count.append(
            Drive.get_encoder_total_count(id[i]))

        # if you want to ommit motor velocity(due to low control rate, for example), comment out script bellow.
        multi_servo_info.motor_velocity.append(
            Drive.get_servo_Velocity(id[i]))

        # if you want to ommit motor current(due to low control rate, for example), comment out script bellow
        multi_servo_info.motor_current.append(
            Drive.get_servo_Current(id[i]))

        multi_servo_info.input_voltage.append(voltage[i])

    multi_servo_info_pub.publish(multi_servo_info)
    del multi_servo_info

    # activate servo_drive_flag again since this process has completed
    servo_drive_flag = 1


# initializing servo process
def initialize_servo_for_velocity_control():
    global num, id
    for i in range(num):  # start resetting and initiating servos again
        Drive.resetServo(id[i])
        Drive.enFreeServo(id[i])
        Drive.reset_encoder_total_count(id[i])
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Drive.change_servocontrol_mode(id[i], 4)


def enfree_servo_after_node_ends(signal, frame):
    global id
    for i in range(num):
        Drive.enFreeServo(id[i])
    sys.exit(0)


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('velocity_control')
    multi_servo_info_pub = rospy.Publisher(
        'multi_servo_info', Multi_servo_info, queue_size=1)

    the_number_of_servo_pub = rospy.Publisher(
        'the_number_of_servo', Int16, queue_size=1, latch=True)

    initial_process()

    rospy.Subscriber('multi_servo_command', Multi_servo_command,
                     callback_servo_command, queue_size=1)

    rospy.spin()
