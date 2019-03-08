#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time
import functions.Kondo as Kondo_B3M

ser = serial.Serial('/dev/ttyUSB0', 1500000)
time.sleep(1)

#set servo mode to FREE mode
ser.write(b'x08\x04\x00\x04\x02\x28\x01\x3B')
print("done1")
time.sleep(0.5)

#set control mode to Pos_control mode
ser.write(b'x08\x04\x00\x04\x02\x28\x01\x3B')
print("done2")
time.sleep(0.5)

#set the motion generate type to EVEN
ser.write(b'x08\x04\x00\x04\x01\x29\x01\x3B')
print("done3")
time.sleep(0.5)

#set gain
ser.write(b'x08\x04\x00\x04\x00\x5C\x01\x6D')
print("done4")
time.sleep(0.5)

#set servo mode to NORMAL mode
ser.write(b'x08\x04\x00\x04\x00\x28\x01\x39')
print("done5")
time.sleep(0.5)

#send SET POSITION command pos:0deg\ time:500ms
#ser.write("\0x09\0x06\0x00\0x00\0x00\0x00\0xF4\0x01\0x04")
ser.write(b'x09\x06\x00\x04\x00\x00\xF4\x01\x08')
time.sleep(1)
print("done 0deg")

#send SET POSITION command pos:45deg\ time:500ms
ser.write(b'x09\x06\x00\x04\x94\x11\xF4\x01\xAD')
time.sleep(1)
print("done 45deg")

#send SET POSITION command pos:0deg\ time:500ms
ser.write(b'x09\x06\x00\x04\x00\x00\xF4\x01\x08')
time.sleep(1)
print("done 0deg")

#send SET POSITION command pos:45deg\ time:500ms
ser.write(b'x09\x06\x00\x04\x94\x11\xF4\x01\xAD')
time.sleep(1)
print("done 45deg")

#set servo mode to FREE mode
ser.write(b'x08\x04\x00\x04\x02\x28\x01\x3B')
print("done free mode")
time.sleep(1)

ser.close()
time.sleep(2)
