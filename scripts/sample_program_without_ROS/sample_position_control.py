#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time, sys, os
sys.path.append(os.pardir)
import Kondo_B3M_functions as Kondo_B3M

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(1)

Kondo_B3M.enFreeServo(4)
Kondo_B3M.change_servocontrol_mode(4, 0)
Kondo_B3M.set_servo_trajectory_to_5Poly(4)
Kondo_B3M.set_servo_gain_to_presets(4, 0)
Kondo_B3M.change_servocontrol_mode(4, 0)

Kondo_B3M.control_servo_by_position_with_time(4, 10000, 500)
Kondo_B3M.get_servo_Position(4)

Kondo_B3M.control_servo_by_position_with_time(4, 0, 500)
Kondo_B3M.get_servo_Position(4)

Kondo_B3M.control_servo_by_position_with_time(4, 10000, 500)
Kondo_B3M.get_servo_Position(4)

Kondo_B3M.control_servo_by_position_with_time(4, 0, 500)
Kondo_B3M.get_servo_Position(4)

Kondo_B3M.enFreeServo(4)


ser.close()
time.sleep(2)
