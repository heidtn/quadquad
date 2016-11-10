#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from quadquad_hardware.msg import QuadServos

import serial
from serial.tools import list_ports
import sys

ser = None

servomap = {
	'BLHip': [0,-1],
	'BRHip': [2, 1],
	'FLHip': [5,-1],
	'FRHip': [6, 1],
	'BLLeg': [1, 1],
	'BRLeg': [3,-1],
	'FLLeg': [4,-1],
	'FRLeg': [7, 1]
}

speed = 20

def set_speed(n, speed):
	#Quick check that things are in range
	if speed > 127 or speed <0:
		speed=1
		print "WARNING: Speed should be between 0 and 127. Setting speed to 1..."
	speed=int(speed)
	#set speed (needs 0x80 as first byte, 0x01 as the second, 0x01 is for speed, 0 for servo 0, and 127 for max speed)
	bud=chr(0x80)+chr(0x01)+chr(0x01)+chr(n)+chr(speed)
	ser.write(bud)

def set_pos(n, angle, direction):
	#Check that things are in range
	if angle > 180 or angle <0:
		angle=90.0
		rospy.loginfo("WARNING: Bad angle. Setting angle to 90 degrees to be safe...")

	#Check direction of limb
	if direction == -1:
		angle = 180 - angle

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
	global ser
	ports = list_ports.comports()
	if(len(ports) == 0):
		print("no com ports found")
		rospy.logerr("ERROR: no com ports found")
		raise
	port = ports[0].device
	port = '/dev/ttyS0' #rpi serial port doesn't enumerate properly on pi3
	ser = serial.Serial(port)
	print("using port: ", port)
	ser.baudrate = 38400	

	for i in servomap:
		set_speed(servomap[i][0], speed)

def handle_msg(servoMsg):
	rospy.loginfo(rospy.get_caller_id() + "new servo command")
	set_pos(servomap['BLHip'][0], servoMsg.BLHip, servomap['BLHip'][1])
	set_pos(servomap['BRHip'][0], servoMsg.BRHip, servomap['BRHip'][1])
	set_pos(servomap['FLHip'][0], servoMsg.FLHip, servomap['FLHip'][1])
	set_pos(servomap['FRHip'][0], servoMsg.FRHip, servomap['FRHip'][1])

	set_pos(servomap['BLLeg'][0], servoMsg.BLLeg, servomap['BLLeg'][1])
	set_pos(servomap['BRLeg'][0], servoMsg.BRLeg, servomap['BRLeg'][1])
	set_pos(servomap['FLLeg'][0], servoMsg.FLLeg, servomap['FLLeg'][1])
	set_pos(servomap['FRLeg'][0], servoMsg.FRLeg, servomap['FRLeg'][1])

def create_listener_node():
	rospy.init_node('quad_servo_controller')
	rospy.Subscriber('servo_controller', QuadServos, handle_msg)

	rospy.spin()
	
	ser.close()
	

if __name__ == "__main__":
	initiate_serial()
	create_listener_node()
