#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 1500000)
time.sleep(1)

#set servo mode to FREE mode
ser.write("\x08\x04\x00\xFE\x02\x28\x01\x37")
print("Set the servos FREE MODE")
print("Set the servos POSITION CONTROL MODE")
time.sleep(0.1)

"""
#set control mode to Pos_control mode
ser.write("\x08\x04\x00\x00\x02\x28\x01\x37")
print("Set the servos POSITION CONTROL MODE")
time.sleep(0.1)
"""

#set the motion generate type to EVEN
ser.write("\x08\x04\x00\xFE\x05\x29\x01\x3B")
print("Set the motion type to 5-poly")
time.sleep(0.1)

#set gain
ser.write("\x08\x04\x00\xFE\x00\x5C\x01\x69")
print("Set the gain as PRESET-0")
time.sleep(0.1)

#set servo mode to NORMAL mode
ser.write("\x08\x04\x00\xFE\x00\x28\x01\x35")
print("Set the servos NORMAL MODE")
time.sleep(0.1)


"""
#send SET POSITION command pos:0deg, time:1000ms
Angle = 0
bAngH = bin(Angle>>8)
bAngL = bin(Angle & 0b11111111)

Time = 500
bTime = bin(Time)
bTimeH = bin(Time>>8)
bTimeL = bin(Time & 0b11111111)

AngL =format(int(bAngL,2),'x')
AngH = format(int(bAngH,2),'x')
TimeL = format(int(bTimeL,2),'x')
TimeH = format(int(bTimeH,2),'x')


bAL = AngL.encode('utf-8')
bAH = AngH.encode('utf-8')
bTL = TimeL.encode('utf-8')
bTH = TimeH.encode('utf-8')

bAL = bytes.fromhex(AngL)
bAH = bytes.fromhex(AngH)
bTL = bytes.fromhex(TimeL)
bTH = bytes.fromhex(TimeH)

print(AngL)
print(AngH)
print(TimeL)
print(TimeH)

SUM = bin((int('09', 16)+int('06', 16)+int('00', 16)+int('00', 16)+int(bAngL,2)+int(bAngH,2)+int(bTimeL,2)+int(bTimeH,2)) & 0b11111111)
xSUM = format(int(SUM,2),'x')
cSUM = xSUM.encode('utf-8')
print(xSUM)
ser.write(b'\x09'+b'\x06'+b'\x00'+b'\x00'+bytes(AngL, 'UTF-8')+bytes(AngH, 'UTF-8')+bytes(TimeL, 'UTF-8')+bytes(TimeH, 'UTF-8')+bytes(xSUM, 'UTF-8')), AngL, AngH, TimeL, TimeH, xSUM
elements = [0x09, 0x06, 0x00, 0x00, 0x00, 0x00, 0xF4, 0x01, 0x04]
var = bytearray(elements)
ser.write(var[0:8])
time.sleep(1)
print("done 0deg")


#send SET POSITION command pos:0deg, time:500ms
ser.write("\x09\x06\x00\x00\x00\x00\xF4\x01\x04")
time.sleep(1)
print("done 0deg")
"""

#send SET POSITION command pos:0deg, time:1000ms
Angle = 4500
bAngH = bin(Angle>>8)
bAngL = bin(Angle & 0b11111111)

Time = 1000
bTime = bin(Time)
bTimeH = bin(Time>>8)
bTimeL = bin(Time & 0b11111111)

AngL =format(int(bAngL,2),'x')
AngH = format(int(bAngH,2),'x')
TimeL = format(int(bTimeL,2),'x')
TimeH = format(int(bTimeH,2),'x')


bAL = AngL.encode()
bAH = AngH.encode()
bTL = TimeL.encode()
bTH = TimeH.encode()

print(AngL)
print(AngH)
print(TimeL)
print(TimeH)

SUM = bin((int('09', 16)+int('06', 16)+int('00', 16)+int('00', 16)+int(bAngL,2)+int(bAngH,2)+int(bTimeL,2)+int(bTimeH,2)) & 0b11111111)
xSUM = format(int(SUM,2),'x')
cSUM = xSUM.encode()
print(xSUM)
bytearray = "\x09\x06\x00\x00" + chr(Angle & 0xff)+chr(Angle >> 8)+chr(Time & 0xff)+chr(Time >> 8)+chr(int(SUM, 2))
print(bytearray)
ser.write(b"bytearray")
time.sleep(1)
print("done 45deg")




#send SET POSITION command pos:0deg, time:1000ms
Angle = 0
bAngH = bin(Angle>>8)
bAngL = bin(Angle & 0b11111111)

Time = 1000
bTime = bin(Time)
bTimeH = bin(Time>>8)
bTimeL = bin(Time & 0b11111111)

AngL =format(int(bAngL,2),'x')
AngH = format(int(bAngH,2),'x')
TimeL = format(int(bTimeL,2),'x')
TimeH = format(int(bTimeH,2),'x')


bAL = AngL.encode()
bAH = AngH.encode()
bTL = TimeL.encode()
bTH = TimeH.encode()

print(AngL)
print(AngH)
print(TimeL)
print(TimeH)

SUM = bin((int('09', 16)+int('06', 16)+int('00', 16)+int('00', 16)+int(bAngL,2)+int(bAngH,2)+int(bTimeL,2)+int(bTimeH,2)) & 0b11111111)
xSUM = format(int(SUM,2),'x')
cSUM = xSUM.encode()
print(xSUM)
bytearray = "\x09\x06\x00\x00" + chr(Angle & 0xff)+chr(Angle >> 8)+chr(Time & 0xff)+chr(Time >> 8)+chr(int(SUM, 2))
print(bytearray)
ser.write(b"bytearray")
time.sleep(1)
print("done 45deg")





#send SET POSITION command pos:0deg, time:500ms
ser.write(b"\x09\x06\x00\x00\x00\x00\xF4\x01\x04")
print("\x09\x06\x00\x00\x00\x00\xF4\x01\x04")
time.sleep(1)
print("done 0deg")

"""
#send SET POSITION command pos:0deg, time:500ms
ser.write("\x09\x06\x00\x00\x00\x00\xF4\x01\x04")
time.sleep(1)
print("done 0deg")

#send SET POSITION command pos:45deg, time:500ms
ser.write("\x09\x06\x00\x00\x94\x11\xF4\x01\xA9")
time.sleep(1)
print("done 45deg")
"""

#set servo mode to FREE mode
ser.write(b"\x08\x04\x00\x00\x02\x28\x01\x37")
print("done free mode")
time.sleep(1)

ser.close()
time.sleep(2)
