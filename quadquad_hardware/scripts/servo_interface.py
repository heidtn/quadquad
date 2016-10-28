#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from quadquad_hardware.msg import QuadServos

import serial
from serial.tools import list_ports
import sys

ser = None


servomap = {
	'BLHip': 0,
	'BRHip': 1,
	'FLHip': 2,
	'FRHip': 3,
	'BLLeg': 4,
	'BRLeg': 5,
	'FLLeg': 6,
	'FRLeg': 7
}

def set_speed(n, speed):
	#Quick check that things are in range
	if speed > 127 or speed <0:
		speed=1
		print "WARNING: Speed should be between 0 and 127. Setting speed to 1..."
	speed=int(speed)
	#set speed (needs 0x80 as first byte, 0x01 as the second, 0x01 is for speed, 0 for servo 0, and 127 for max speed)
	bud=chr(0x80)+chr(0x01)+chr(0x01)+chr(n)+chr(speed)
	ser.write(bud)

def set_pos(n, angle):
	#Check that things are in range
	if angle > 180 or angle <0:
		angle=90
		rospy.loginfo("WARNING: Bad angle. Setting angle to 90 degrees to be safe...")

	#Valid range is 500-5500
	offyougo=int(5000*angle/180)+500
	#Get the lowest 7 bits
	byteone=offyougo&127
	#Get the highest 7 bits
	bytetwo=(offyougo-(offyougo&127))/128
	#move to an absolute position in 8-bit mode (0x04 for the mode, 
	#0 for the servo, 0-255 for the position (spread over two bytes))
	bud=chr(0x80)+chr(0x01)+chr(0x04)+chr(n)+chr(bytetwo)+chr(byteone)
	ser.write(bud)

def initiate_serial():
	ports = list_ports.comports()
	if(len(ports) == 0):
		print("no com ports found")
		rospy.logerr("ERROR: no com ports found")
		raise
	ser = serial.Serial(0)
	ser.baudrate = 38400	

def handle_msg(servoMsg):
	rospy.loginfo(rospy.get_caller_id() + "new servo command")
	set_pos(servomap['BLHip'], servoMsg.BLHip)
	set_pos(servomap['BRHip'], servoMsg.BRHip)
	set_pos(servomap['FLHip'], servoMsg.FLHip)
	set_pos(servomap['FRHip'], servoMsg.FRHip)

	set_pos(servomap['BLLeg'], servoMsg.BLLeg)
	set_pos(servomap['BRLeg'], servoMsg.BRLeg)
	set_pos(servomap['FLLeg'], servoMsg.FLLeg)
	set_pos(servomap['FRLeg'], servoMsg.FRLeg)

def create_listener_node():
	rospy.init_node('quad_servo_controller')
	rospy.Subscriber('servo_controller', QuadServos, handle_msg)

	rospy.spin()
	
	ser.close()
	

if __name__ == "__main__":
	initiate_serial()
	create_listener_node()
