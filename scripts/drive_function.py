#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import math
import sys

accumulated_position_error = 0

# open serial port
# you have to modify device name according to your environment, or fix device name as follows by symbolic link.
ser = serial.Serial('/dev/Kondo_USB-RS485_converter', 1500000)


# initialize (= reset) servo whose id is "ID"
def initServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x02 + 0x28 + 0x01) & 0b11111111
    enFreeServo_command = []
    enFreeServo_command += [chr(0x08), chr(0x04), chr(0x00),
                            chr(ID), chr(0x02), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(enFreeServo_command)
    # print("set servo ID:" + str(ID) + " to FREE mode")
    time.sleep(0.002)  # wait until this process done
    if ser.inWaiting() == 5:
        ret = 1
    else:
        ret = 0
    return ret


# reset servo whose id is "ID"
def resetServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x06 + 0x05 + 0x00 + ID + 0x02) & 0b11111111
    resetServo_command = []
    resetServo_command += [chr(0x06), chr(0x05),
                           chr(0x00), chr(ID), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(resetServo_command)
    time.sleep(0.1)  # wait until this process done
    print("Reset servo ID:" + str(ID))


# enfree servo whose id is "ID"
def enFreeServo(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x02 + 0x28 + 0x01) & 0b11111111
    enFreeServo_command = []
    enFreeServo_command += [chr(0x08), chr(0x04), chr(0x00),
                            chr(ID), chr(0x02), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(enFreeServo_command)
    time.sleep(0.1)  # wait until this process done
    print("set servo ID:" + str(ID) + " to FREE mode")


# IDが"ID"なサーボを位置制御モード、スタンバイにする関数（軌道生成：別途指定、　制御ゲイン：プリセット#0）
# mode : 00>positionCTRL, 04>velocityCTRL, 08>current(torque)CTRL, 12>feedforwardCTRL
def change_servocontrol_mode(ID, mode):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + mode + 0x28 + 0x01) & 0b11111111
    change_servocontrol_mode_command = []
    change_servocontrol_mode_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(mode), chr(0x28), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(change_servocontrol_mode_command)
    time.sleep(0.1)  # wait until this process done

    if mode == 0:
        set_servo_gain_to_presets(ID, 0)
        print("set servo ID:" + str(ID) +
              " to position control mode with preset gain #0")
    elif mode == 4:
        set_servo_gain_to_presets(ID, 1)
        print("set servo ID:" + str(ID) +
              " to velocity control mode with preset gain #1")
    elif mode == 8:
        set_servo_gain_to_presets(ID, 2)
        print("set servo ID:" + str(ID) +
              " to current(torque) control mode with preset gain #2")
    elif mode == 12:
        print("set servo ID:" + str(ID) + " to feed-forward control mode")


# IDが"ID"なサーボの位置制御モード時の軌道生成を5-polyモードにする関数
def set_servo_trajectory_to_5Poly(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x05 + 0x29 + 0x01) & 0b11111111
    set_servo_trajectory_to_5Poly_command = []
    set_servo_trajectory_to_5Poly_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(0x05), chr(0x29), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_trajectory_to_5Poly_command)
    time.sleep(0.1)  # wait until this process done

    print("set servo ID:" + str(ID) + " to 5-poly Trajectory")


# IDが"ID"なサーボの位置制御モード時の軌道生成をEVENモード（等速）にする関数
def set_servo_trajectory_to_EVEN(ID):
    ser.reset_input_buffer()  # flush serial buffer before starting this process
    SUM = (0x08 + 0x04 + 0x00 + ID + 0x01 + 0x29 + 0x01) & 0b11111111
    set_servo_trajectory_to_EVEN_command = []
    set_servo_trajectory_to_EVEN_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(0x01), chr(0x29), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_trajectory_to_EVEN_command)
    time.sleep(0.1)  # wait until this process done

    print("set servo ID:" + str(ID) + " to Even Trajectroy")


# IDが"ID"なサーボの制御ゲインをプリセットのものに設定する関数
# プリセット0:位置制御用、1:速度制御用、2:トルク制御用
def set_servo_gain_to_presets(ID, PresetNumber):
    ser.reset_input_buffer()  # 返信データを読み取ってバッファから消しておく
    SUM = (0x08 + 0x04 + 0x00 + ID + PresetNumber + 0x5c + 0x01) & 0b11111111
    set_servo_gain_to_presets_command = []
    set_servo_gain_to_presets_command += [chr(0x08), chr(0x04), chr(
        0x00), chr(ID), chr(PresetNumber), chr(0x5c), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(set_servo_gain_to_presets_command)
    time.sleep(0.1)  # wait until this process done


# IDが"ID"なサーボの位置を、目標時間"Time(ms)"をかけて"Angle(/100 deg)"にセットする関数
def control_servo_by_position_with_time(ID, target_position, Time_msec):
    # 目標角度が負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
    if target_position < 0:
        modAngle = 65536 + target_position
    else:
        modAngle = target_position

    SUM = (0x09 + 0x06 + 0x00 + ID + (modAngle & 0xff) + (modAngle
                                                          >> 8) + (Time_msec & 0xff) + (Time_msec >> 8)) & 0b11111111
    control_servo_by_position_with_time_command = []
    control_servo_by_position_with_time_command += [chr(0x09), chr(0x06), chr(0x00), chr(ID), chr(
        modAngle & 0xff), chr(modAngle >> 8), chr(Time_msec & 0xff), chr(Time_msec >> 8), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(control_servo_by_position_with_time_command)
    time.sleep(1.0 * Time_msec / 1000)
    print("set servo ID:" + str(ID) + " to position " +
          str(target_position / 100) + "[deg] by " + str(Time_msec) + "[ms]")


# IDが"ID"なサーボの位置を"Angle(/100 deg)"にセットする関数、余裕時間として"Time[ms]"を見ておく
# 軌道生成を行わないので急峻な動きになる。
# 移動に要する時間は関数呼び出し側で確保する必要あり。
def control_servo_by_position_without_time(ID, target_position):
    # 目標角度が負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
    if target_position < 0:
        target_position = 65536 + target_position

    SUM = (0x09 + 0x04 + 0x00 + ID + (target_position & 0xff) +
           (target_position >> 8) + 0x2A + 0x01) & 0b11111111
    control_servo_by_position_without_time_command = []
    control_servo_by_position_without_time_command += [chr(0x09), chr(0x04), chr(0x00), chr(
        ID), chr(target_position & 0xff), chr(target_position >> 8), chr(0x2A), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(control_servo_by_position_without_time_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 5:
            ser.reset_input_buffer()  # 返信データを読み取ってバッファから消しておく
            break
    # print("set servo ID:" + str(ID) + " to position " +
    #       str(target_position / 100.0) + "[deg]")


def control_servo_by_Position_multicast(args):
    num_of_servos = len(args) / 2
    id = []
    position_command = []
    id_sum = 0
    position_command_sum = 0

    for i in range(num_of_servos):
        id.append(args[i])
        position_command.append(args[i + num_of_servos])
        if position_command[i] < 0:
            position_command[i] = 65536 + position_command[i]
        id_sum = id_sum + id[i]
        position_command_sum = position_command_sum + \
            (position_command[i] & 0xff) + (position_command[i] >> 8)

    command_length = (3 + 3 * num_of_servos + 3) & 0b11111111
    SUM = (command_length + 0x04 + 0x00 + id_sum +
           position_command_sum + 0x2A + num_of_servos) & 0b11111111

    control_servo_by_Position_multicast_command = []
    control_servo_by_Position_multicast_command += [
        chr(command_length), chr(0x04), chr(0x00)]

    for j in range(num_of_servos):
        control_servo_by_Position_multicast_command += [
            chr(id[j]), chr(position_command[j] & 0xff), chr(position_command[j] >> 8)]

    control_servo_by_Position_multicast_command += [
        chr(0x2A), chr(num_of_servos), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(control_servo_by_Position_multicast_command)

    # wait for a certain seconds since multicast mode wouldn't reply anything to you
    time.sleep(0.0015)
    args = []


def control_servo_by_Velocity(ID, target_velocity):  # velocity(100*deg/sec)
    # 目標角度が負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
    if target_velocity < 0:
        target_velocity = 65536 + target_velocity

    SUM = (0x09 + 0x04 + 0x00 + ID + (target_velocity & 0xff) +
           (target_velocity >> 8) + 0x30 + 0x01) & 0b11111111
    control_servo_by_Velocity_command = []
    control_servo_by_Velocity_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(
        target_velocity & 0xff), chr(target_velocity >> 8), chr(0x30), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(control_servo_by_Velocity_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 5:
            ser.reset_input_buffer()  # 返信データを読み取ってバッファから消しておく
            break
    # print("set servo ID:" + str(ID) + " to Velocity " +
    #       str(target_velocity / 100.0) + "[deg/sec]")


# function to control servo by velocity [*0.01 deg/sec] at multicast mode
def control_servo_by_Velocity_multicast(args):
    num_of_servos = len(args) / 2
    id = []
    velocity_command = []
    id_sum = 0
    velocity_command_sum = 0

    for i in range(num_of_servos):
        id.append(args[i])
        velocity_command.append(args[i + num_of_servos])
        if velocity_command[i] < 0:
            velocity_command[i] = 65536 + velocity_command[i]
        id_sum = id_sum + id[i]
        velocity_command_sum = velocity_command_sum + \
            (velocity_command[i] & 0xff) + (velocity_command[i] >> 8)

    command_length = (3 + 3 * num_of_servos + 3) & 0b11111111
    SUM = (command_length + 0x04 + 0x00 + id_sum +
           velocity_command_sum + 0x30 + num_of_servos) & 0b11111111

    control_servo_by_Velocity_multicast_command = []
    control_servo_by_Velocity_multicast_command += [
        chr(command_length), chr(0x04), chr(0x00)]

    for j in range(num_of_servos):
        control_servo_by_Velocity_multicast_command += [
            chr(id[j]), chr(velocity_command[j] & 0xff), chr(velocity_command[j] >> 8)]

    control_servo_by_Velocity_multicast_command += [
        chr(0x30), chr(num_of_servos), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(control_servo_by_Velocity_multicast_command)

    # wait for a certain seconds since multicast mode wouldn't reply anything to you
    time.sleep(0.0015)
    args = []


# IDが"ID"なサーボの目標トルクを"Torque(mNm)"にセットする関数
def control_servo_by_Torque(ID, Torque_mNm):
    # now = time.time()
    if Torque_mNm < 0:  # 目標トルクが負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
        modTorque = 65536 + Torque_mNm
    else:  # 目標トルクが正の場合でも、コンソールにTorque値を表示したいので、信号送信用の変数はmodTorqueとする
        modTorque = Torque_mNm
    SUM = (0x09 + 0x04 + 0x00 + ID + (modTorque & 0xff) +
           (modTorque >> 8) + 0x3c + 0x01) & 0b11111111
    control_servo_by_Torque_command = []
    control_servo_by_Torque_command += [chr(0x09), chr(0x04), chr(0x00), chr(
        ID), chr(modTorque & 0xff), chr(modTorque >> 8), chr(0x3c), chr(0x01), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(control_servo_by_Torque_command)

    # wait until reply will come
    while True:
        if ser.inWaiting() == 5:
            ser.reset_input_buffer()  # 返信データを読み取ってバッファから消しておく
            break
    # print results
    # print("set servo ID:" + str(ID)
    #       + " to Torque " + str(Torque_mNm) + "[mNm]")


def control_servo_by_Torque_multicast(args):
    # now = time.time()
    length = len(args) / 2
    id = []
    torque_command = []
    id_sum = 0
    torque_command_sum = 0
    for i in range(length):
        id.append(args[i])
        torque_command.append(args[i + length])
        if torque_command[i] < 0:
            torque_command[i] = 65536 + torque_command[i]
        id_sum = id_sum + id[i]
        torque_command_sum = torque_command_sum + \
            (torque_command[i] & 0xff) + (torque_command[i] >> 8)

    command_length = (3 + 3 * length + 3) & 0b11111111
    SUM = (command_length + 0x04 + 0x00 + id_sum +
           torque_command_sum + 0x3c + length) & 0b11111111

    control_servo_by_Torque_multicast_command = []
    control_servo_by_Torque_multicast_command += [
        chr(command_length), chr(0x04), chr(0x00)]
    for j in range(length):
        control_servo_by_Torque_multicast_command += [
            chr(id[j]), chr(torque_command[j] & 0xff), chr(torque_command[j] >> 8)]
    control_servo_by_Torque_multicast_command += [
        chr(0x3c), chr(length), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(control_servo_by_Torque_multicast_command)
    time.sleep(0.0015)
    args = []


def control_servo_position_by_Torque(ID, target_position):
    global accumulated_position_error
    Kp = 0.6
    Kd = 0.03
    Ki = 0.0003
    # print(ID)
    current_position = get_servo_Position(ID)
    current_velocity = get_servo_Velocity(ID)
    accumulated_position_error = accumulated_position_error + \
        (target_position - current_position)
    if accumulated_position_error > 100000:
        accumulated_position_error = 100000
    elif accumulated_position_error < -100000:
        accumulated_position_error = -100000
    target_torque = (target_position - current_position) * \
        Kp - (current_velocity) * Kd + accumulated_position_error * Ki
    target_torque = int(target_torque)
    if target_torque > 32000:
        target_torque = 32000
    elif target_torque < -32000:
        target_torque = -32000
    # print(target_torque)
    control_servo_by_Torque(ID, target_torque)


# IDが"ID"なサーボの角度取得
def get_servo_Position(ID):
    # アドレス0x2cから2バイト分（=角度）読みだす信号を作成し、送信
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x2c + 0x02) & 0b11111111
    get_servo_Position_command = []
    get_servo_Position_command += [chr(0x07), chr(0x03),
                                   chr(0x00), chr(ID), chr(0x2c), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(get_servo_Position_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    # 返信を処理。最初の４バイトは共通なので、適当な変数に格納しておく。次の２バイトが角度なので、受信し、リトルエンディアンで整数に変換。
    Receive = ser.read(4)
    Angle1 = ser.read(1)
    Angle2 = ser.read(1)
    intAngle1 = ord(Angle1)
    intAngle2 = ord(Angle2)

    Angle = (intAngle2 << 8) | intAngle1

    # 角度が正の場合は角度*100の値が表示されるが、負の場合は違うので、そこを処理
    if Angle > 0x8300:
        Angle = Angle - 0x10000

    # 角度を返す
    # print(str(Angle / 100.0) + "[deg]")
    return Angle


# IDが"ID"なサーボの速度取得
def get_servo_Velocity(ID):
    # アドレス0x2cから2バイト分（=角度）読みだす信号を作成し、送信
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x32 + 0x02) & 0b11111111
    get_servo_Velocity_command = []
    get_servo_Velocity_command += [chr(0x07), chr(0x03),
                                   chr(0x00), chr(ID), chr(0x32), chr(0x02), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(get_servo_Velocity_command)
    # wait until receive the data (3 bytes at minimum)
    while True:
        if ser.inWaiting() == 7:
            break

    # process the reply from servo. first 4 bytes is general information, so you can discard it.
    # next 2 bytes are motor velocity, so get them and merge by little endian
    Receive = ser.read(4)
    Velocity1 = ser.read(1)
    Velocity2 = ser.read(1)
    intVelocity1 = ord(Velocity1)
    intVelocity2 = ord(Velocity2)

    Velocity = (intVelocity2 << 8) | intVelocity1

    # 角度が正の場合は角度*100の値が表示されるが、負の場合は違うので、そこを処理
    if Velocity > 0x8300:
        Velocity = Velocity - 0x10000

    # 角度を返す
    # print(str(Velocity / 100.0) + "[deg/sec]")
    return Velocity


# サーボ電流値を読み取る関数
def get_servo_Current(ID):
    # アドレス0x48から2バイト分（=電流値）読みだす信号を作成し、送信
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x48 + 0x02) & 0b11111111
    get_servo_Current_command = []
    get_servo_Current_command += [chr(0x07), chr(0x03),
                                  chr(0x00), chr(ID), chr(0x48), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(get_servo_Current_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    Current1 = ser.read(1)
    Current2 = ser.read(1)
    intCurrent1 = ord(Current1)
    intCurrent2 = ord(Current2)
    Current = (intCurrent2 << 8) | intCurrent1

    # 電流値が正の場合はその値が表示されるが、負の場合は違うので、そこを処理
    if Current > 0x8300:
        Current = Current - 0x10000

    # return current
    # print(str(Current) + " [mA]")
    return Current


# サーボにかかる電圧を読み取る関数
def get_servo_voltage(ID):
    # アドレス0x4aから2バイト分（=電流値）読みだす信号を作成し、送信
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x4a + 0x02) & 0b11111111
    get_servo_voltage_command = []
    get_servo_voltage_command += [chr(0x07), chr(0x03),
                                  chr(0x00), chr(ID), chr(0x4a), chr(0x02), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(get_servo_voltage_command)
    # wait until receive the data (3 bytes at minimum)
    while True:
        if ser.inWaiting() > 3:
            break

    # process the reply from servo. first 4 bytes is general information, so you can discard it.
    # next 2 bytes are voltage, so get them and merge by little endian
    Receive = ser.read(4)
    voltage1 = ser.read(1)
    voltage2 = ser.read(1)
    int_voltage1 = ord(voltage1)
    int_voltage2 = ord(voltage2)
    voltage = (int_voltage2 << 8) | int_voltage1

    # return voltage
    # print(str(voltage) + " [mV]")
    return voltage


# サーボ内MUCの温度を読み取る関数
def get_mcu_temperature(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x44 + 0x02) & 0b11111111
    get_mcu_temperature_command = []
    # アドレス0x44から2バイト分（=電流値）読みだす信号を作成し、送信
    get_mcu_temperature_command += [chr(0x07), chr(0x03),
                                    chr(0x00), chr(ID), chr(0x44), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(get_mcu_temperature_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    mcu_temperature1 = ser.read(1)
    mcu_temperature2 = ser.read(1)
    int_mcu_temperature1 = ord(mcu_temperature1)
    int_mcu_temperature2 = ord(mcu_temperature2)
    mcu_temperature = (int_mcu_temperature2 << 8) | int_mcu_temperature1

    # ondoが正の場合はその値が表示されるが、負の場合は違うので、そこを処理
    if mcu_temperature > 0x8300:
        mcu_temperature = mcu_temperature - 0x10000

    # return mcu_temperature
    mcu_temperature = mcu_temperature / 100.0
    print(str(mcu_temperature) + " [degree_celcius]")
    return mcu_temperature


# サーボ本体温度を読み取る関数
def get_servo_temperature(ID):
    # アドレス0x46から2バイト分（=電流値）読みだす信号を作成し、送信
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x46 + 0x02) & 0b11111111
    get_servo_temperature_command = []
    get_servo_temperature_command += [chr(0x07), chr(0x03),
                                      chr(0x00), chr(ID), chr(0x46), chr(0x02), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(get_servo_temperature_command)
    # 通信が来るまで待つ
    while True:
        if ser.inWaiting() == 7:
            break

    Receive = ser.read(4)
    servo_temperature1 = ser.read(1)
    servo_temperature2 = ser.read(1)
    int_servo_temperature1 = ord(servo_temperature1)
    int_servo_temperature2 = ord(servo_temperature2)
    servo_temperature = (int_servo_temperature2 << 8) | int_servo_temperature1

    # ondoが正の場合はその値が表示されるが、負の場合は違うので、そこを処理
    if servo_temperature > 0x8300:
        servo_temperature = servo_temperature - 0x10000

    # return mcu_temperature
    servo_temperature = servo_temperature / 100.0
    print(str(servo_temperature) + " [degree_celcius]")
    return servo_temperature


# サーボのエンコーダカウントをリセットする関数
def reset_encoder_total_count(ID):
    SUM = (0x0B + 0x04 + 0x00 + ID + 0x00 + 0x00 +
           0x00 + 0x00 + 0x52 + 0x01) & 0b11111111
    reset_encoder_total_count_command = []
    reset_encoder_total_count_command += [chr(0x0B), chr(0x04), chr(0x00), chr(
        ID), chr(0x00), chr(0x00), chr(0x00), chr(0x00), chr(0x52), chr(0x01), chr(SUM)]

    ser.reset_input_buffer()  # flush serial buffer before starting this process
    ser.write(reset_encoder_total_count_command)
    time.sleep(0.1)  # wait until this process done
    print("reset encoder")


# サーボのエンコーダカウントを読み取る関数
def get_encoder_total_count(ID):
    SUM = (0x07 + 0x03 + 0x00 + ID + 0x52 + 0x04) & 0b11111111
    get_encoder_total_count_command = []
    get_encoder_total_count_command += [chr(0x07), chr(
        0x03), chr(0x00), chr(ID), chr(0x52), chr(0x04), chr(SUM)]

    # flush input buffer before sending something
    ser.reset_input_buffer()
    ser.write(get_encoder_total_count_command)

    # # wait until receive the data (4 bytes at minimum)
    while True:
        if ser.inWaiting() == 9:
            break

    # process the reply from servo. first 4 bytes is general information, so you can discard it.
    # next 4 bytes are encoder count, so get them and merge by little endian
    Receive = ser.read(4)
    EncoderCount1 = ser.read(1)
    EncoderCount2 = ser.read(1)
    EncoderCount3 = ser.read(1)
    EncoderCount4 = ser.read(1)

    EncoderCount1 = ord(EncoderCount1)
    EncoderCount2 = ord(EncoderCount2)
    EncoderCount3 = ord(EncoderCount3)
    EncoderCount4 = ord(EncoderCount4)

    EncoderCount = (EncoderCount4 << 24) | (EncoderCount3
                                            << 16) | (EncoderCount2 << 8) | EncoderCount1
    if EncoderCount >= 2147483648:
        EncoderCount = EncoderCount - 4294967296
    # カウント値を返す
    # print(str(EncoderCount) + "[count]")
    return EncoderCount
