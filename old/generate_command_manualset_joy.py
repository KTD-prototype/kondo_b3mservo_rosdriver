#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import rospy
import Kondo_B3M_functions as Kondo_B3M
from sensor_msgs.msg import Joy
from kondo_b3mservo_rosdriver.msg import Multi_servo_command

initial_process_flag = 1
initial_setparam_flag = 1
num = 0
target_position = []
target_velocity = []
target_torque = []


def set_the_num_of_servo():
    global num
    if rospy.has_param('num_of_servo'):
        num = rospy.get_param('num_of_servo')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the number of servos. Plsease command '$rosparam set /num_of_servo THE_NUMBER_OF_SERVOS'")
    try:
        if num < 0:
            raise Exception()
    except:
        rospy.logerr("value error: the number of servos")
        sys.exit(1)
    return num


def callback_generate_multi_command(joy_msg):
    global target_position, target_velocity, target_torque, num, initial_process_flag
    multi_servo_command = Multi_servo_command()

    if initial_process_flag == 1:
        global num
        for i in range(num):
            target_position.append(0)
            target_velocity.append(0)
            target_torque.append(0)
        initial_process_flag = 0

    for i in range(num):
        target_position[i] = joy_msg.axes[0] * 32000  # left stick LR
        target_velocity[i] = joy_msg.axes[3] * 32767  # right stick LR
        target_torque[i] = joy_msg.axes[1] * 7000  # left stick FB

        multi_servo_command.target_position.append(target_position[i])
        multi_servo_command.target_velocity.append(target_velocity[i])
        multi_servo_command.target_torque.append(target_torque[i])
    pub.publish(multi_servo_command)
    del multi_servo_command


if __name__ == '__main__':
    rospy.init_node('generate_multi_command')
    # node_name = rospy.get_name()
    # param_name_num = node_name + "/num_of_servo"
    # param_name_id = node_name+"/multi_servo_id"
    if initial_setparam_flag == 1:
        set_the_num_of_servo()
        initial_setparam_flag = 0

    rospy.Subscriber('joy', Joy, callback_generate_multi_command, queue_size=5)
    pub = rospy.Publisher('multi_servo_command',
                          Multi_servo_command, queue_size=5)

    rospy.spin()
