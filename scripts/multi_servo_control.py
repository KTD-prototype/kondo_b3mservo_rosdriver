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

# global parameters
SERVO_ID = []  # servo IDs which you are controlling
CONTROL_MODE = []  # servo control mode : position, velocity, torque

# parameters for monitoring battery voltage[mV]
BATTERY_VOLTAGE_WARN = 11200
BATTERY_VOLTAGE_FATAL = 10700
voltage_monitor_count = 0
servo_reset_flag = False
servo_drive_flag = True

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)


# function that is called at initiation of this process, to configure the system and initialize servos
def initial_process():
    global SERVO_ID, CONTROL_MODE

    # get system configuration as ROS_params
    SERVO_ID = rospy.get_param('~servo_id')  # IDs of servos those are connected, configured manually as an array
    CONTROL_MODE = rospy.get_param('~control_mode')  # control modes of each servos, configured manually as an array

    # get the number of servos that are connected, and publish it as ROS message, to inform other node
    the_number_of_servo = len(SERVO_ID)
    the_number_of_servo_pub.publish(the_number_of_servo)

    # scan control modes of all servos. if there's invalid value, publish an alert.
    # to avoid alerting multiple times, introduce check flag.
    parameter_check_flag = False
    for i in range(len(SERVO_ID)):
        if CONTROL_MODE[i] != 0 and CONTROL_MODE[i] != 4 and CONTROL_MODE[i] != 8 and CONTROL_MODE[i] != 16:
            if parameter_check_flag == False:
                rospy.logfatal(
                    "there're something wrong in your ros_param : control_mode. They must be 0, 4, 8, or 16, for position, velocity, torque, and position_by_torque")
                parameter_check_flag = True

    # publish information of servos under control.
    rospy.loginfo("you are controlling a servo whose ID is : " + str(SERVO_ID) +
                  " at control mode : " + str(CONTROL_MODE) + " where 0:positon, 4:velocity, 8:torque, 16:position by torque")

    # initialize all servos
    initialize_servo()


def callback_servo_command(multi_servo_command):
    global SERVO_ID, CONTROL_MODE, servo_reset_flag, servo_drive_flag

    # prepare parameters as array for targets of control.
    # each array should includes target values equivalent to tha total number of servos.
    target_position = [] * len(SERVO_ID)
    target_velocity = [] * len(SERVO_ID)
    target_torque = [] * len(SERVO_ID)
    target_position_by_torque = [] * len(SERVO_ID)

    # drive servos only when the flag is active
    if servo_drive_flag == True:
        # get servo command from ROS message
        target_position = multi_servo_command.target_position
        target_velocity = multi_servo_command.target_velocity
        target_torque = multi_servo_command.target_torque
        target_position_by_torque = multi_servo_command.target_position_by_torque

        # drive each servos
        for i in range(len(SERVO_ID)):
            if CONTROL_MODE[i] == 0:
                Drive.control_servo_by_position_without_time(SERVO_ID[i], target_position[i])
            elif CONTROL_MODE[i] == 4:
                Drive.control_servo_by_Velocity(SERVO_ID[i], target_velocity[i])
            elif CONTROL_MODE[i] == 8:
                Drive.control_servo_by_Torque(SERVO_ID[i], target_torque[i])
            elif CONTROL_MODE[i] == 16:
                Drive.control_servo_position_by_Torque(SERVO_ID[i], target_position_by_torque[i])

        # publish information of servos as ROS message
        publish_servo_info()

    # if flag for resetting servos is active
    if servo_reset_flag == True:
        initialize_servo()
        rospy.logwarn('complete resetting servos!')
        servo_reset_flag = False  # deactivate flag for resetting servos

        # deactivate flag for servo locomoting until it's triggered to restart
        servo_drive_flag = False


# function to get information from all servos, and publish them as ROS message
def publish_servo_info():
    global SERVO_ID
    global servo_drive_flag, voltage_monitor_count

    # deactivate servo_drive_flag to prevent from sending velocity command to the servos before completing this process
    servo_drive_flag = False

    # set up an instance for ROS message to be published
    multi_servo_info = Multi_servo_info()

    # prepare paramter to store battery voltage
    battery_voltage = [0] * len(SERVO_ID)

    # Don't have to monitor voltage at every loop, so get sparsed at a time per a certain loops
    # monitor per 100 cycles
    if voltage_monitor_count == 100:
        voltage_monitor_count = 0  # reset count

        # prepare flag paramter whether to warn voltage or not
        voltage_fatal_flag = False
        voltage_warn_flag = False

        # survey voltages of all servos (frequently those are same since they're connected to same battery)
        for i in range(len(SERVO_ID)):
            battery_voltage[i] = Drive.get_servo_voltage(SERVO_ID[i])
            if battery_voltage[i] < BATTERY_VOLTAGE_FATAL:  # if the voltage is lower than a certain fatal value
                voltage_fatal_flag = True
            elif battery_voltage[i] < BATTERY_VOLTAGE_WARN:  # if the voltage is lower than a certain warning value
                voltage_warn_flag = True

        # if more than one of values of voltage are lower than threshold, then display warning to the console.
        if voltage_fatal_flag == True:
            rospy.logfatal('battery voltage is fatally low !')
            voltage_fatal_flag = False
        elif voltage_warn_flag == True:
            rospy.logwarn('battery voltage is low !')
            voltage_warn_flag = False

    # increment count to execute process above periodically
    voltage_monitor_count = voltage_monitor_count + 1

    # contain information to ROS message
    for i in range(len(SERVO_ID)):
        multi_servo_info.encoder_count.append(Drive.get_encoder_total_count(SERVO_ID[i]))
        # if you want to ommit motor velocity(due to low control rate, for example), comment out script bellow.
        multi_servo_info.motor_velocity.append(Drive.get_servo_Velocity(SERVO_ID[i]))
        multi_servo_info.motor_current.append(Drive.get_servo_Current(SERVO_ID[i]))
        multi_servo_info.input_voltage.append(battery_voltage[i])

    # publish ROS message
    multi_servo_info_pub.publish(multi_servo_info)
    del multi_servo_info

    # activate servo_drive_flag again since this process has completed
    servo_drive_flag = 1


# initializing servo process
def initialize_servo():
    global SERVO_ID, CONTROL_MODE
    local_control_mode = []  # parameters to indicates control modes for each servos

    for i in range(len(SERVO_ID)):
        # reset servo before setting its control mode
        Drive.resetServo(SERVO_ID[i])
        Drive.enFreeServo(SERVO_ID[i])
        Drive.reset_encoder_total_count(SERVO_ID[i])

        # set servo control mode : position, velocity, torque, or position_by_torque
        local_control_mode.append(CONTROL_MODE[i])  # transcript control mode from it's global parameter to local
        local_control_mode = list(local_control_mode)  # convert local parameter into "list" before modify them

        # if control mode indicates "16", it means servo must be in control mode : position_by_torque.
        # then, servo itself should be controlled based on torque.
        if local_control_mode[i] == 16:
            local_control_mode[i] = 8
        Drive.change_servocontrol_mode(SERVO_ID[i], local_control_mode[i])


def enfree_servo_after_node_ends(signal, frame):
    global id
    for i in range(num):
        Drive.enFreeServo(id[i])
    sys.exit(0)


signal.signal(signal.SIGINT, enfree_servo_after_node_ends)


if __name__ == '__main__':
    rospy.init_node('single_servo_control')
    multi_servo_info_pub = rospy.Publisher(
        'multi_servo_info', Multi_servo_info, queue_size=1)

    the_number_of_servo_pub = rospy.Publisher(
        'the_number_of_servo', Int16, queue_size=1, latch=True)

    initial_process()

    rospy.Subscriber('multi_servo_command', Multi_servo_command,
                     callback_servo_command, queue_size=1)

    rospy.spin()
