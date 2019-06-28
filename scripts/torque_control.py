#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import rospy
import serial
import time
import signal
import sys
from std_msgs.msg import Int16, Int8
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info
import Kondo_B3M_functions as Kondo_B3M

target_torque = []
id = []
merged_command = []
num = 0

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 11200  # 11000[mV] at 3cell LiPo battery
BATTERY_VOLTAGE_FATAL = 10700   # 10500[mV] at 3cell LiPo battery (3.5V/cell)
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
    global target_torque, voltage

    for i in range(255):  # investigate only ID:0 to 10 to shorten time to detect connected servos
        result = Kondo_B3M.initServo(i)
        if result == 1:
            id.append(i)
            num = num + 1

    initialize_servo_for_torque_control()
    for j in range(num):
        target_torque.append(0)
        voltage.append(12000)

    print("")
    rospy.logwarn("you are controlling [ " + str(num) + " ] servos whose IDs is : " + str(
        id) + " at TORQUE CONTROL MODE, which are automatically detected.")

    target_torque = list(target_torque)
    the_number_of_servo_pub.publish(num)


def callback_servo_command(multi_servo_command):
    global num, target_torque, id, merged_command
    global servo_reset_flag, servo_drive_flag

    # actual locomotions are only conducted when the flag is active
    if servo_drive_flag == 1:
        target_torque = multi_servo_command.target_torque
        target_torque = list(target_torque)
        for i in range(num):
            merged_command.append(id[i])
        for j in range(num):
            merged_command.append(target_torque[j])
        Kondo_B3M.control_servo_by_Torque_multicast(merged_command)
        publish_servo_info()
        merged_command = []

    # if flag for resetting servos is active
    if servo_reset_flag == 1:
        initialize_servo_for_torque_control()
        rospy.logwarn('complete resetting servos!')
        servo_reset_flag = 0  # deactivate flag for resetting servos

        # deactivate flag for servo locomoting until it's triggered to restart
        servo_drive_flag = 0


def publish_servo_info():
    global id, num, voltage
    global servo_drive_flag
    global battery_voltage_warn_flag, battery_voltage_fatal_flag, voltage_monitor_flag

    # deactivate servo_drive_flag to prevent from sending torque command to the servos before completing this process
    servo_drive_flag = 0

    multi_servo_info = Multi_servo_info()

    # Don't have to monitor voltage at every loop, so get sparsed at a time per a certain loops
    # monitor per 100 cycles
    if voltage_monitor_flag == 100:
        voltage_monitor_flag = 0
        for j in range(num):
            voltage[j] = Kondo_B3M.get_servo_voltage(id[j])
            if voltage[j] < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
                print("")
                rospy.logwarn('battery voltage is low !')
                battery_voltage_warn_flag = 1
            elif voltage[j] < BATTERY_VOLTAGE_FATAL and battery_voltage_fatal_flag == 0:
                print("")
                rospy.logfatal('battery voltage is fatally low !')
    voltage_monitor_flag = voltage_monitor_flag + 1
    voltage = list(voltage)

    for i in range(num):
        multi_servo_info.encoder_count.append(
            Kondo_B3M.get_encoder_total_count(id[i]))

        # if you want to ommit motor velocity(due to low control rate, for example), comment out script bellow.
        multi_servo_info.motor_velocity.append(
            Kondo_B3M.get_servo_Velocity(id[i]))

        # if you want to ommit motor current(due to low control rate, for example), comment out script bellow
        multi_servo_info.motor_current.append(
            Kondo_B3M.get_servo_Current(id[i]))

        multi_servo_info.input_voltage.append(voltage[i])

    multi_servo_info_pub.publish(multi_servo_info)
    del multi_servo_info

    # activate servo_drive_flag again since this process has completed
    servo_drive_flag = 1


# callback function to trigger servo reset
def callback_servo_reset(trigger):
    global servo_reset_flag
    if trigger.data == 1:  # if flag is active
        servo_drive_flag = 0  # deactivate servo driving flag to stop locomoting servos
        servo_reset_flag = 1  # activate flag to reset servo
    else:
        pass


# callback function to trigger restarting servos
def callback_servo_drive(trigger):
    global servo_drive_flag
    if trigger.data == 1 and servo_drive_flag == 0:  # if flag is active and servos are not been locomoted
        servo_drive_flag = 1  # activate servo driving flag
        rospy.logwarn('restarting servos!')
    else:
        pass


# initializing servo process
def initialize_servo_for_torque_control():
    global num, id
    for i in range(num):  # start resetting and initiating servos again
        Kondo_B3M.resetServo(id[i])
        Kondo_B3M.enFreeServo(id[i])
        Kondo_B3M.reset_encoder_total_count(id[i])
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Kondo_B3M.change_servocontrol_mode(id[i], 8)


# enfreeing servo process for end of the script
def enfree_servo_after_node_ends(signal, frame):
    global id, num
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
                     callback_servo_command, queue_size=1)
    rospy.Subscriber('reset_trigger', Int8, callback_servo_reset, queue_size=1)
    rospy.Subscriber('drive_trigger', Int8, callback_servo_drive, queue_size=1)

    rospy.spin()
    # rate = rospy.Rate(50)
    # while not rospy.is_shutdown():
    #     try:
    #         torque_control()
    #     except IOError:
    #         pass
    #     rate.sleep()
