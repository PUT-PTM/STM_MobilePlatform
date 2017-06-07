# STM_MobilePlatform
LineFollower “Sławomir”

# Overview
Mobile platform controlled via Bluetooth/Uart. It has two options - line follower and manual control.

# Description
Following the line based on the PD algorithm, all options are available from smartphone application RoboRemoFree.
Used modules:
Microcontroller STM32F407 Discovery
Engine DG01D-A130
Bluetooth module 2.1 XM-15B
Infrared sensor KAmodQTR8A
	
# Tools
Software: CooCox CoIDE Version: 1.7.8
Language: C
RoboRemoFree on Android

# How to run 
Upload program to STM32F4, connect all needed pins that are given in main.c file. Download RoboRemoFree on Android system and configure it
by using commands from main.c file

# How to compile 
CoIDE to compile code and send it to flash memory.

# Future improvements 
Manual control can be improved by changing bluetooth commands. Ultrawave sensor configuration.

# License
Distributed under MIT license.

# Credits 
Project created by Paulina Mrozek, Kamil Sagalara.
Computer Science, Faculty of Electrical Engineering, Poznan University of Technology.
The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.

# Supervisor: Adam Bondyra
 
 
 
