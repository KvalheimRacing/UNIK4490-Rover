#!/usr/bin/env python

import rospy
import rover_motordriver.msg as msgs

def main():
	rospy.loginfo("Node initializing")
	rospy.init_node('autopilot_test', anonymous=True)
	rospy.loginfo("Setting up subscriptions and publishing")
	pub = rospy.Publisher('/cmd_vel', msgs.Velocity, queue_size=10)
	rospy.loginfo("Initialized")
	
	rate = rospy.Rate(0.25)
	
	
	vel_change_rate = -1.0
	left_vel = 0.0
	right_vel = 0.0
	while not rospy.is_shutdown(): 
		if abs(left_vel) > 30:
			vel_change_rate *= -1.0
			
		left_vel += vel_change_rate
		right_vel -= vel_change_rate
		
		msg = msgs.Velocity()
		msg.left_vel = 20
		msg.right_vel = 20
		
		#msg = msgs.Position()
		#msg.position.x = 20
		#msg.position.y = 0
		
		pub.publish(msg)
		
		rate.sleep()
		
		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

