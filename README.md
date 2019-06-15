# kondo-b3mservo-rosdriver
　　
## Overview
ROS package for control of servo motor by ***Kondo Kagaku Inc***.  
<br>
<br>
<br>
## Description
This package is for control of serial servo (B3Mseries) by ***Kondo Kagaku Inc*** via ROS.  
It includes nodes for send/receive commands to/from servos and pub/sub them, an additional file to wrap functions to generate commands to servos, and peripherals.(a sample file to control by a joystick, etc.)  
<br>
<br>
<br>
## Demo
sorry, still in prepare
<br>
<br>
<br>
## Requirements
confirmed environment is as follows:
  * Ubuntu16.04  
  * python2.7.12  
  * ROS kinetic kame  
<br>
<br>
<br>

## Install
`cd ~/NAME_OF_YOUR_ROS_WORKSPACE(e.g catkin_ws)/src`
`git clone git@github.com:k24koba/kondo-b3mservo-rosdriver.git`  
`cd ~/catkin_ws`  
`catkin_make`  
<br>
<br>
<br>
## How to use
in directory ***scripts***, you have several files:
  * generate_command_autodetect_joy.py
  * Kondo_B3M_functions.py
  * Kondo_B3M_functions.pyc
  * position_control_autodetect.py
  * torque_control_autodetect.py
  * torque_control_autodetect_multicast.py
  * velocity_control_autodetect.py
