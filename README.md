# kondo-b3mservo-controller-ros  
## Overview  
ROS package for control of servo motor by Kondo Kagaku inc.  
## Description  
This package is for control of serial servo (B3Mseries) by Kondo Kagaku inc via ROS.  
It includes nodes for send/receive commands to/from servos and pub/sub them, an additional file to wrap functions to generate commands to servos, and peripherals.(a sample file to control by a joystick, etc.)  
## Demo  

## Requirements
Ubuntu16.04  
python2.7.12  
ROS kinetic kame  

## Usage  
## Install
'git clone git@github.com:k24koba/kondo-b3mservo-roscontrol.git'  
'cd ~/**name of ROS workspace(for example, catkin_ws)**'  
'catkin_make'  
