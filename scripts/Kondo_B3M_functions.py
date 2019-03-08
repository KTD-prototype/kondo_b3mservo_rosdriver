#!/usr/bin/env python
# -*- coding: utf-8 -*-
#code for python2
import serial
import time
import math
import struct

ser = serial.Serial('/dev/ttyUSB0', 1500000)

#IDが"ID"なサーボをリセットする関数
def resetServo(ID):
	SUM = (0x06 + 0x05 + 0x00 + ID + 0x02) & 0b11111111
	resetServo_command = []
	resetServo_command += [chr(0x06), chr(0x05), chr(0x00), chr(ID), chr(0x02), chr(SUM)]
	ser.write(resetServo_command)
	print("Reset servo ID:" + str(ID))

#IDが"ID"なサーボをフリーにする関数
def enFreeServo(ID):
	SUM = (0x08 + 0x04 + 0x00 + ID + 0x02 + 0x28 + 0x01) & 0b11111111
	enFreeServo_command = []
	enFreeServo_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(0x02), chr(0x28), chr(0x01), chr(SUM)]
	ser.write(enFreeServo_command)
	print("set servo ID:" + str(ID) + " to FREE mode")


#IDが"ID"なサーボを位置制御モード、スタンバイにする関数（軌道生成：別途指定、　制御ゲイン：プリセット#0）
def set_servo_to_PositionCtrlMode(ID):
	enFreeServo(ID)	#モード変更前にサーボをフリーにする
	set_servo_gain_to_presets(ID, 0)
	SUM = (0x08 + 0x04 + 0x00 + ID + 0x00 + 0x28 + 0x01) & 0b11111111
	set_servo_to_PositionCtrlMode_command = []
	set_servo_to_PositionCtrlMode_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(0x00), chr(0x28), chr(0x01), chr(SUM)]
	ser.write(set_servo_to_PositionCtrlMode_command)
	print("set servo ID:" + str(ID) + " to Pos Ctrl mode, Gain:preset#0")


#IDが"ID"なサーボの位置制御モード時の軌道生成を5-polyモードにする関数
def set_servo_trajectory_to_5Poly(ID):
	SUM = (0x08 + 0x04 + 0x00 + ID + 0x05 + 0x29 + 0x01) & 0b11111111
	set_servo_trajectory_to_5Poly_command = []
	set_servo_trajectory_to_5Poly_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(0x05), chr(0x29), chr(0x01), chr(SUM)]
	ser.write(set_servo_trajectory_to_5Poly_command)
	print("set servo ID:" + str(ID) + " to 5-poly Trajectory")


#IDが"ID"なサーボの位置制御モード時の軌道生成をEVENモード（等速）にする関数
def set_servo_trajectory_to_EVEN(ID):
	SUM = (0x08 + 0x04 + 0x00 + ID + 0x01 + 0x29 + 0x01) & 0b11111111
	set_servo_trajectory_to_EVEN_command = []
	set_servo_trajectory_to_EVEN_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(0x01), chr(0x29), chr(0x01), chr(SUM)]
	ser.write(set_servo_trajectory_to_EVEN_command)
	print("set servo ID:" + str(ID) + " to Even Trajectroy")


#IDが"ID"なサーボをトルク制御モードにする関数（制御ゲイン：プリセット#2）
def set_servo_to_TorqueCtrlMode(ID):
	set_servo_gain_to_presets(ID, 2)
	SUM = (0x08 + 0x04 + 0x00 + ID + 0x08 + 0x28 + 0x01) & 0b11111111
	set_servo_to_TorqueCtrlMode_command = []
	set_servo_to_TorqueCtrlMode_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(0x08), chr(0x28), chr(0x01), chr(SUM)]
	ser.write(set_servo_to_TorqueCtrlMode_command)
	print("set servo ID:" + str(ID) + " to Trq Ctrl mode, Gain:preset#2")


#IDが"ID"なサーボの制御ゲインをプリセットのものに設定する関数
#プリセット0:位置制御用、1:速度制御用、2:トルク制御用
def set_servo_gain_to_presets(ID, PresetNumber):
	SUM = (0x08 + 0x04 + 0x00 + ID + PresetNumber + 0x5c + 0x01) & 0b11111111
	set_servo_gain_to_presets_command = []
	set_servo_gain_to_presets_command += [chr(0x08), chr(0x04), chr(0x00), chr(ID), chr(PresetNumber), chr(0x5c), chr(0x01), chr(SUM)]
	ser.write(set_servo_gain_to_presets_command)
	if PresetNumber == 0:
		print("set Ctrl-gains of servo ID:" + str(ID) + " for Position Control")
	if PresetNumber == 1:
		print("set Ctrl-gains of servo ID:" + str(ID) + " for Velocity Control")
	if PresetNumber == 2:
		print("set Ctrl-gains of servo ID:" + str(ID) + " for Torque Control")
	#ser.flush()


#IDが"ID"なサーボの位置を、目標時間"Time(ms)"をかけて"Angle(/100 deg)"にセットする関数
def control_servo_by_position_with_time(ID, Angle_centDeg, Time_msec):
	if Angle_centDeg < 0:	#目標角度が負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
		modAngle = 65536 + Angle_centDeg
	else:
		modAngle = Angle_centDeg

	SUM = (0x09 + 0x06 + 0x00 + ID + (modAngle&0xff) + (modAngle>>8) + (Time_msec&0xff) + (Time_msec>>8)) & 0b11111111
	control_servo_by_position_with_time_command = []
	control_servo_by_position_with_time_command += [chr(0x09), chr(0x06), chr(0x00), chr(ID), chr(modAngle&0xff), chr(modAngle>>8), chr(Time_msec&0xff), chr(Time_msec>>8), chr(SUM)]
	ser.write(control_servo_by_position_with_time_command)
	time.sleep(1.0*Time_msec/1000)
	print("set servo ID:" + str(ID) + " to position " + str(Angle_centDeg/100) + "[deg] by " + str(Time_msec) + "[ms]")


#IDが"ID"なサーボの位置を"Angle(/100 deg)"にセットする関数、余裕時間として"Time[ms]"を見ておく
#軌道生成を行わないので急峻な動きになる。
def control_servo_by_position_without_time(ID, Angle_centDeg):
	if Angle_centDeg < 0:	#目標角度が負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
		modAngle = 65536 + Angle_centDeg
	else:		#目標角度が正の場合でも、コンソールにAngle値を表示したいので、信号送信用の変数はmodAngleとする
		modAngle = Angle_centDeg

	SUM = (0x09 + 0x04 + 0x00 + ID + (modAngle&0xff) + (modAngle>>8) + 0x2A + 0x01) & 0b11111111
	control_servo_by_position_without_time_command = []
	control_servo_by_position_without_time_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(modAngle&0xff), chr(modAngle>>8), chr(0x2A), chr(0x01), chr(SUM)]
	ser.write(control_servo_by_position_without_time_command)
	time.sleep(1)
	print("set servo ID:" + str(ID) + " to position " + str(Angle_centDeg/100) + "[deg]")


##IDが"ID"なサーボの目標トルクを"Torque(mNm)"にセットする関数
def control_servo_by_Torque(ID, Torque_mNm):
	if Torque_mNm < 0:	#目標トルクが負の場合、-1→65535(0xffff)、-32000→33536(0x8300)と変換
		modTorque = 65536 + Torque_mNm
	else:		#目標トルクが正の場合でも、コンソールにTorque値を表示したいので、信号送信用の変数はmodTorqueとする
		modTorque = Torque_mNm
	SUM = (0x09 + 0x04 + 0x00 + ID + (modTorque&0xff) + (modTorque>>8) + 0x3c + 0x01) & 0b11111111
	control_servo_by_Torque_command = []
	control_servo_by_Torque_command += [chr(0x09), chr(0x04), chr(0x00), chr(ID), chr(modTorque&0xff), chr(modTorque>>8), chr(0x3c), chr(0x01), chr(SUM)]
	ser.write(control_servo_by_Torque_command)
	#time.sleep(2)
	print("set servo ID:" + str(ID) + " to Torque " + str(Torque_mNm) +"[mNm]")



#IDが"ID"なサーボの角度取得
def read_servo_Position(ID):
	#何か信号を送る度にサーボが返信を返してきており、それがバッファに溜まっているので、全てクリアする
	ser.reset_input_buffer()

	#アドレス0x2cから2バイト分（=角度）読みだす信号を作成し、送信
	SUM = (0x07 + 0x03 + 0x00 + ID + 0x2c + 0x02) & 0b11111111
	read_servo_Position_command = []
	read_servo_Position_command += [chr(0x07), chr(0x03), chr(0x00), chr(ID), chr(0x2c), chr(0x02), chr(SUM)]
	ser.write(read_servo_Position_command)

	#通信が来るまで待つ(1us)
	time.sleep(0.000001)

	#返信を処理。最初の４バイトは共通なので、適当な変数に格納しておく。次の２バイトが角度なので、受信し、リトルエンディアンで整数に変換。
	Receive = ser.read(4)
	Ang1 = ser.read(1)
	Ang2 = ser.read(2)
	Ang1 = ord(Ang1)
	Ang2 = ord(Ang2)

	#Ang = int.from_bytes(Ang, 'little') python3.2以降で使える記述
	#Ang = int(Ang.encode('hex'), 16)
	#Ang = struct.unpack("<i", Ang)


	#角度が正の場合は角度*100の値が表示されるが、負の場合は違うので、そこを処理
	if Ang > 0x8300:
		Ang = Ang - 0x10000

	#角度を返す
	print(str(Ang/100) + "[deg]")
	return Ang


"""
#IDが"ID"なサーボのトルク制御（角度フィードバック, 目標角度"goal_Angle"）
def Trq_by_Ang(ID, goal_Angle):
	#トルクゲインを決定
	Kp = 0.2	#比例ゲイン
	Ki = 0.3	#積分ゲイン
	Kd = 0.3	#微分ゲイン
	count = 10	#トルクの立ち上がりを穏やかにする係数（その１）
	count2 = 0	#トルクの立ち上がりを穏やかにする係数（その２、穏やかさが長続きするように）
	count3 = 0	#制御終了判定をするための変数
	count4 = 0	#offsetをキャンセルする場合に使う変数（積分制御用）
	current_Angle = readServoPOS(ID)
	previous_Angle = current_Angle
	offset = 0	#累積角度誤差（積分制御用）
	current_time = time.time()	#実行時間（角度フィードバックトルク制御の開始時点）
	previous_time = current_time
	Torque_MAX = 700	#目標トルクの最大値を規定（mNm)

	while(True):
		#トルクの立ち上がりを穏やかにする変数の処理
		count2 = count / 10
		count += 1

		#一定周期ごとに積分制御用の累積角度誤差をゼロにする（外乱を受けつつ一定角度に保持するような場合に発散してしまうので）
		#count4 += 1
		if count4 == 100:
			offset = 0
			count4 = 0

		#現在角度を取得
		previous_Angle = current_Angle
		current_Angle = readServoPOS(ID)

		#位置オフセットを積分により取得
		previous_time = current_time
		current_time = time.time()
		dt = current_time - previous_time
		if math.fabs(goal_Angle - current_Angle) < 500:
			offset += (goal_Angle - current_Angle) * dt

		#目標角度との差分に応じたトルクを発揮（PID制御）
		Torque = (goal_Angle - current_Angle)*Kp + (previous_Angle - current_Angle)*Kd + offset * Ki

		#そもそもトルクが大きすぎるとサーボに負荷がかかるので、制限する。
		if Torque > Torque_MAX:
			Torque = Torque_MAX
		if Torque < -1 * Torque_MAX:
			Torque = -1 * Torque_MAX

		#トルクの立ち上がりを穏やかにする係数をかける
		#トルクは整数しか受け取れないので、変換
		Torque = Torque * (1 - (1 / count2))
		Torque = int(Torque)


		#目標トルク値をコンソールに出力しつつ、実際にトルク制御
		#print(str(current_Angle/100) + '[deg], ' + str(Torque) + '[mNm]')
		#print(count2)
		#print(offset)
		setTRQ(ID, Torque)
		time.sleep(0.002)

		#角度誤差が一定以下の時間が一定期間続いたら、制御終了
		if math.fabs(goal_Angle - current_Angle) < 100:	#単位：[/100 deg]
			count3 += 1
			if count3 > 10:
				setTRQ(ID, 0)	#トルクをゼロにして制御をやめる。
				break
	print("end of Positioning by Torque Ctrl")
"""


#IDが"ID"なサーボのトルク制御（角度フィードバック, 目標角度"goal_Angle"）、無限ループなし（実質P制御）
def Trq_by_Ang(ID, goal_Angle):
	#トルクゲインを決定
	Kp = 0.2	#比例ゲイン
	Ki = 0.0	#積分ゲイン
	Kd = 0.0	#微分ゲイン
	count = 10	#トルクの立ち上がりを穏やかにする係数（その１）
	count2 = 0	#トルクの立ち上がりを穏やかにする係数（その２、穏やかさが長続きするように）
	count3 = 0	#制御終了判定をするための変数
	count4 = 0	#offsetをキャンセルする場合に使う変数（積分制御用）
	current_Angle = readServoPOS(ID)
	previous_Angle = current_Angle
	offset = 0	#累積角度誤差（積分制御用）
	current_time = time.time()	#実行時間（角度フィードバックトルク制御の開始時点）
	previous_time = current_time
	Torque_MAX = 700	#目標トルクの最大値を規定（mNm)


	#トルクの立ち上がりを穏やかにする変数の処理
	count2 = count / 10
	count += 1

	#一定周期ごとに積分制御用の累積角度誤差をゼロにする（外乱を受けつつ一定角度に保持するような場合に発散してしまうので）
	#count4 += 1
	if count4 == 100:
		offset = 0
		count4 = 0

	#現在角度を取得
	previous_Angle = current_Angle
	current_Angle = readServoPOS(ID)
	#位置オフセットを積分により取得
	previous_time = current_time
	current_time = time.time()
	dt = current_time - previous_time
	if math.fabs(goal_Angle - current_Angle) < 500:
		offset += goal_Angle - current_Angle

	#目標角度との差分に応じたトルクを発揮（PID制御）
	Torque = (goal_Angle - current_Angle)*Kp + (previous_Angle - current_Angle)*Kd + offset * Ki * dt

	#そもそもトルクが大きすぎるとサーボに負荷がかかるので、制限する。
	if Torque > Torque_MAX:
		Torque = Torque_MAX
	if Torque < -1 * Torque_MAX:
		Torque = -1 * Torque_MAX

	#トルクの立ち上がりを穏やかにする係数をかける
	#トルクは整数しか受け取れないので、変換
	#Torque = Torque * (1 - (1 / count2))
	Torque = int(Torque)


	#目標トルク値をコンソールに出力しつつ、実際にトルク制御
	print(str(current_Angle/100) + '[deg], ' + str(Torque) + '[mNm]')
	#print(count2)
	#print(offset)
	setTRQ(ID, Torque)
	time.sleep(0.002)

	#角度誤差が一定以下の時間が一定期間続いたら、制御終了
	if math.fabs(goal_Angle - current_Angle) < 100:	#単位：[/100 deg]
		count3 += 1
		if count3 > 10:
			setTRQ(ID, 0)	#トルクをゼロにして制御をやめる。
	#print("end of Positioning by Torque Ctrl")
