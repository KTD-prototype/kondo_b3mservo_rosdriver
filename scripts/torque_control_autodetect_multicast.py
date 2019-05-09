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
ramped_target_torque = []
pre_target_torque = []
id = []
merged_command = []
num = 0
initial_process_flag = 1
found_servo_flag = 1
MINIMUM_STEP_OF_TARGET_TORQUE = 100

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200
BATTERY_VOLTAGE_FATAL = 13800
voltage = []

voltage_monitor_flag = 0
servo_reset_flag = 0
servo_drive_flag = 1

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


def initial_process():
    global id, num, initial_process_flag, the_number_of_servo_pub
    global target_torque, ramped_target_torque, pre_target_torque, voltage

    # for i in range(255):
    for i in range(10):
        # Kondo_B3M.resetServo(i)
        result = Kondo_B3M.initServo(i)
        # print(result)
        if result == 1:
            id.append(i)
            num = num + 1

    for j in range(num):
        Kondo_B3M.resetServo(id[j])
        Kondo_B3M.enFreeServo(id[j])
        Kondo_B3M.reset_encoder_total_count(id[j])
        # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
        Kondo_B3M.change_servocontrol_mode(id[j], 8)

        target_torque.append(0)
        ramped_target_torque.append(0)
        pre_target_torque.append(0)
        voltage.append(16000)

    print("")
    rospy.logwarn("you are controlling [ " + str(num) + " ] servos whose IDs is : " + str(
        id) + " at TORQUE CONTROL MODE, which are automatically detected.")

    target_torque = list(target_torque)
    ramped_target_torque = list(ramped_target_torque)
    pre_target_torque = list(pre_target_torque)
    initial_process_flag = 0
    the_number_of_servo_pub.publish(num)


def callback_servo_command(multi_servo_command):
    global num, target_torque, pre_target_torque, ramped_target_torque, id, merged_command, servo_reset_flag, servo_drive_flag
    target_torque = multi_servo_command.target_torque
    target_torque = list(target_torque)
    # print(pre_target_torque)
    # print(target_torque)

    for i in range(num):
        # ramp target torque since drastic difference of target torque may cause lock of servo
        ramped_target_torque[i] = ramp_target_torque(
            target_torque[i], pre_target_torque[i])

    for j in range(num):
        merged_command.append(id[j])
    for k in range(num):
        merged_command.append(ramped_target_torque[k])

    # actual locomotions are only conducted when the flag is active
    if servo_drive_flag == 1:
        Kondo_B3M.control_servo_by_Torque_multicast(merged_command)

    pre_target_torque = ramped_target_torque
    publish_servo_info()
    merged_command = []

    # if flag for resetting servos is active
    if servo_reset_flag == 1:
        for j in range(num):  # start resetting and initiating servos again
            Kondo_B3M.resetServo(id[j])
            Kondo_B3M.enFreeServo(id[j])
            Kondo_B3M.reset_encoder_total_count(id[j])
            # mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
            Kondo_B3M.change_servocontrol_mode(id[j], 8)
        rospy.logwarn('complete resetting servos!')
        servo_reset_flag = 0  # deactivate flag for resetting servos

        # deactivate flag for servo locomoting until it's triggered to restart
        servo_drive_flag = 0


def publish_servo_info():
    global id, num, battery_voltage_warn_flag, battery_voltage_fatal_flag, voltage, voltage_monitor_flag
    multi_servo_info = Multi_servo_info()

    for i in range(num):
        voltage = list(voltage)
        multi_servo_info.encoder_count.append(
            Kondo_B3M.get_encoder_total_count(id[i]))

        # ommit to get motor velocity due to low control rate
        # multi_servo_info.motor_velocity.append(
        #     Kondo_B3M.get_servo_Velocity(id[i]))

        multi_servo_info.input_voltage.append(voltage[i])

    # Don't have to monitor voltage at every loop, so get sparsed at a time per a certain loops
    # monitor per 500 cycles (cycles usually at over 50Hz)
    if voltage_monitor_flag % 500 == 0:
        voltage_monitor_flag = 1
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
    multi_servo_info_pub.publish(multi_servo_info)
    del multi_servo_info


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


def enfree_servo_after_node_ends(signal, frame):
    global id, num
    for i in range(num):
        Kondo_B3M.enFreeServo(id[i])
    sys.exit(0)


def ramp_target_torque(torque_command, previous_torque_command):
    if abs(torque_command) > abs(previous_torque_command) + MINIMUM_STEP_OF_TARGET_TORQUE:
        if torque_command * previous_torque_command < 0 and abs(previous_torque_command) > MINIMUM_STEP_OF_TARGET_TORQUE:
            torque_command = 0
        elif torque_command > 0:
            torque_command = previous_torque_command + MINIMUM_STEP_OF_TARGET_TORQUE
        elif torque_command < 0:
            torque_command = previous_torque_command - MINIMUM_STEP_OF_TARGET_TORQUE
    elif torque_command * previous_torque_command < 0 and abs(previous_torque_command) > MINIMUM_STEP_OF_TARGET_TORQUE:
        torque_command = 0
    return torque_command


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
