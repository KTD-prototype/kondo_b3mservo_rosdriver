# kondo-b3mservo-rosdriver
　　
## Overview
ROS package for control of servo motor by ***Kondo Kagaku Inc***.  
　　
## Description
This package is for control of serial servo (B3Mseries) by ***Kondo Kagaku Inc*** via ROS.  
It includes nodes for send/receive commands to/from servos and pub/sub them, an additional file to wrap functions to generate commands to servos, and peripherals.(a sample file to control by a joystick, etc.)  
　　
## Demo
sorry, still in prepare
　　
## Requirements
confirmed environment is as follows:
  * Ubuntu16.04  
  * python2.7.12  
  * ROS kinetic kame  
　　
## Install
'git clone git@github.com:k24koba/kondo-b3mservo-roscontrol.git'  
'cd ~/**name of ROS workspace(for example, catkin_ws)**'  
'catkin_make'  
