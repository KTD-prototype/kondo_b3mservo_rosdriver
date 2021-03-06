# kondo-b3mservo-rosdriver
<!-- TOC depthFrom:1 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [kondo-b3mservo-rosdriver](#kondo-b3mservo-rosdriver)
	- [Overview](#overview)
	- [Description](#description)
	- [Demonstration](#demonstration)
	- [Basic Requirements](#basic-requirements)
	- [Install](#install)
	- [Before use](#before-use)
	- [How to use](#how-to-use)
		- [General descriptions](#general-descriptions)
		- [Manual control by publishing command message directly from a terminal](#manual-control-by-publishing-command-message-directly-from-a-terminal)
		- [Manual control by joystick (game pad)](#manual-control-by-joystick-game-pad)
		- [Getting servo information](#getting-servo-information)

<!-- /TOC -->

## Overview
ROS package for control of servo motor by ***Kondo Kagaku Inc***.<br>

<br>
<br>
<br>

## Description
This package is for control of serial servo (B3Mseries) by ***Kondo Kagaku Inc*** via ROS.<br>
It includes nodes for send/receive commands to/from servos and pub/sub them, an additional file to wrap functions to generate commands to servos, and peripherals.(a sample file to control by a joystick, etc.)
<br>
<br>
<br>

## Demonstration
Demonstration video to introduce position control by manual publishing, and torque control by joy stick.<br>
![result](https://github.com/KTD-prototype/kondo_b3mservo_rosdriver/blob/media/sample.gif?raw=true)
<br>
<br>
<br>

## Basic Requirements
confirmed environment is as follows:
  * Ubuntu16.04
  * python2.7.12
  * ROS kinetic kame
* servo motor : [B3M-SC-1170-A](https://kondo-robot.com/product/03092)
* serial interface between servo and PC : [RS485-USB adapter](https://kondo-robot.com/product/02133)
* power source : 3 cell Lithium Polymer Battery
<br>
<br>

## Install
`   $ cd ~/NAME_OF_YOUR_ROS_WORKSPACE(e.g. catkin_ws)/src`<br>
`   $ git clone git@github.com:k24koba/kondo_b3mservo_rosdriver.git`<br>
`   $ cd ~/catkin_ws`<br>
`   $ catkin_make`
<br>
<br>
<br>
## Before use
Confirm that you have connected your B3M servo to your PC via [RS485-USB adapter](https://kondo-robot.com/product/02133)
First, you have to go through several process to use serial servos by ***Kondo Kagaku Inc***.<br>
You can consult official website about this process(https://kondo-robot.com/faq/usb_adapter_for_linux_2019)
<br>
<br>
<br>
## How to use
### General descriptions
in directory ***scripts***, you can see several files and each descriptions are as bellow:
  * ***generate_command_autodetect_joy.py***  : command generator to your servo from ROS joystick package
  * ***Kondo_B3M_functions.py***  :  collection of functions to generate servo command
  * ***position_control.py***  : node to control servos by its position (angle)
  * ***torque_control.py***  : node to control servos by its torque
  * ***velocity_control.py*** : node to control servos by its velocity
<br>
Each nodes (for position, velocity, torque control) will scan your serial port you set in the node, and automatically identify how many servos are you connected and what is their IDs are.<br>
After process above, it will immediately get ready to receive your command via ROS message!<br>
Command will be an array of integer number ordered by their IDs, which indicates target position[*0.01deg] / velocity[*0.01deg/sec] / torque[mNm].<br>
For example, if you send position command : [4500], then your servo will move to Angle 45 [deg]. If you send [4500, -3000] to 2 servos with IDs: 1, 2, then your servo with ID:1 will move to angle 45 [deg], and your servo with ID:2 will move to angle -30 [deg].
After receiving your command, the node will return servo information via ROS message.
<br>

### Manual control by publishing command message directly from a terminal
As simple way to use, execute arbitrary node file (position, velocity, or torque control)

e.g.<br>
`		$ roscore`
<br>(in another terminal, suppose you want to control by position,)<br>
`		$ python position_control.py`

You may have an error such as ***serial.serialutil.SerialException: [Errno 2] could not open port /dev/Kondo_USB-RS485_converter: [Errno 2] No such file or directory: '/dev/Kondo_USB-RS485_converter'***
<br>
To solve this you can take two ways : <br>
  * change device name : ***/dev/Kondo_USB-RS485_converter*** in ***NODE_FILE.py*** and ***Kondo_B3M_functions.py*** to appropriate name such as ***/dev/ttyUSB0***
  * change and fix the device name recognized by your PC by using symbolic link
<br>
Former option is more easier, but Later may effective since sometimes your device name automatically recognized by your PC will changed through rebooting or reconnecting.
After fixing those error, try again to run the node.
<br>

e.g.<br>
`		$ python position_control.py`

the script will automatically detect and recognize how many servo are you connected, and IDs of each servos.
Then, you can send your command! (supposing you are running node for position control)
<br>

e.g.(at another terminal, in case you are controlling ***2*** servos)<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_position:[1000, 1000]}"`<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_velocity:[1000, 1000]}"`<br>
`		$ rostopic pub /multi_servo_command kondo_b3mservo_rosdriver/Multi_servo_command "{target_torque:[500, 500]}"`<br>

Which type of message to publish depends on which control mode did you selected (which script are you running.)

<br>

### Manual control by joystick (game pad)
If you want to control servo by your joy stick, you can use the launch files (suppose you already installed ROS package for joystick) :
<br>

`		$ roslaunch kondo_b3mservo_rosdver position_control_sample.launch`<br>
`		$ roslaunch kondo_b3mservo_rosdver velocity_control_sample.launch`<br>
`		$ roslaunch kondo_b3mservo_rosdver torque_control_sample.launch`<br>

Joy stick assignment is as follows (May be you should press other botton to start controlling via joystick):
 * position control : left and right of LEFT JOY STICK
 * velocity control : left and right of RIGHT JOY STICK
 * torque control : up and down of LEFT JOY STICK
<br>


### Getting servo information
If you want to see servo information, command as follows in another terminal:
`		$ rostopic echo /multi_servo_info`
<br>
Then you can see
 * encoder_count [count]
 * input_voltage (battery voltage) [mV]
 * motor_velocity [*0.01 deg/sec]
 * motor_current [mA]
