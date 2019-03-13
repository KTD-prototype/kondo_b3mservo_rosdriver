#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time
import Kondo_B3M_functions_forsampleprogram as Kondo_B3M

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(1)

Kondo_B3M.resetServo(4)
Kondo_B3M.enFreeServo(4)

Kondo_B3M.change_servocontrol_mode(4, 4) #mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL

Kondo_B3M.control_servo_by_Velocity(4, 9000)
time.sleep(1)
Kondo_B3M.get_servo_Velocity(4)

Kondo_B3M.control_servo_by_Velocity(4, -9000)
time.sleep(1)
Kondo_B3M.get_servo_Velocity(4)

Kondo_B3M.enFreeServo(4)

ser.close()
time.sleep(1)
