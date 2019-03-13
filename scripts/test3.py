#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time
import Kondo_B3M_functions as Kondo_B3M

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(1)

Kondo_B3M.enFreeServo(4)
Kondo_B3M.reset_encoder_total_count(4)
Kondo_B3M.change_servocontrol_mode(4, 8)#mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL

i = 0
while(i<50):
    Kondo_B3M.control_servo_by_Torque(4, 200)
    Kondo_B3M.get_encoder_total_count(4)
    i = i + 1
while(i<150):
    Kondo_B3M.control_servo_by_Torque(4, -200)
    Kondo_B3M.get_encoder_total_count(4)
    i = i + 1

Kondo_B3M.enFreeServo(4)


ser.close()
