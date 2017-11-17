#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from manual_test import set_speed, FORWARD, BACKWARD, WHEELS_RIGHT, WHEELS_LEFT

import serial

serial_socket = None
got_message = False

def message_handler(message):
	print "message_handler()"
	global serial_socket, got_message
	got_message = True
	print message.linear 
	
		
	right_wheels = 20*message.linear.x + 30*message.angular.z
	left_wheels = 20*message.linear.x - 30*message.angular.z
	
	set_speed(serial_socket, FORWARD, right_wheels)
	set_speed(serial_socket, FORWARD, left_wheels)
	
def talker():
	global serial_socket, got_message
	rospy.init_node('talker', anonymous=True)
	
	rospy.Subscriber("/cmd_vel", Twist, message_handler)

	serial_socket =  serial.Serial(port='/dev/ttyUSB0', baudrate=9600)
		
	rate = rospy.Rate(20) # 1hz
	
	print "Talker is set up"
	
	while not rospy.is_shutdown():
		if got_message == False:
			print "No message"
			set_speed(serial_socket, WHEELS_RIGHT, 40)
			set_speed(serial_socket, WHEELS_LEFT, 40)
		else:
			print "GOT MESSAGE"
			got_message = False
		rate.sleep()
		
	set_speed(serial_socket, WHEELS_RIGHT,0)
	set_speed(serial_socket, WHEELS_LEFT,0)
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
