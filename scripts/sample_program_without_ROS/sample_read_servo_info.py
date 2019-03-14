#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time, sys, os
sys.path.append(os.pardir)
import Kondo_B3M_functions as Kondo_B3M

ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)
time.sleep(0.1)

Kondo_B3M.read_current_limit(4)
Kondo_B3M.get_servo_Current(4)


ser.close()
time.sleep(0.1)
