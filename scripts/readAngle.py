#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time
import Kondo_B3M_functions as Kondo_B3M

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(1)

Kondo_B3M.enFreeServo(4)

Kondo_B3M.read_servo_Position(4)

Kondo_B3M.enFreeServo(4)


ser.close()
time.sleep(2)
