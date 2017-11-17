#!/usr/bin/env python
import rospy
import numpy as np
import serial
import tf
from tf import TransformBroadcaster
from nav_msgs.msg import Odometry as OdomMsg
from geometry_msgs.msg import * 

import rover_motordriver.msg as msgs

class Odometry(object):
    def __init__(self):	
        rospy.init_node('odometry', anonymous=True)
        
        self.imu_socket =  serial.Serial(port='/dev/ttyACM1', baudrate=115200)
		
        
        rospy.Subscriber("/est_vel", msgs.Velocity , self.velocity_handler)
		#rospy.Subscriber("/cmd_angle", msgs.Angle, self.angle_handler)
		#rospy.Subscriber("/cmd_dist", msgs.Distance, self.distance_handler)
		#rospy.Subscriber("/cmd_pos", msgs.Position, self.position_handler)

    
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        
        self.x = 0.0
        self.y = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        
        self.th = 0.0
        
        self.current_time = rospy.get_rostime();
        self.last_time = 0
    def velocity_handler(self, message):
        self.vx = (message.left_vel + message.right_vel)/2.0
        self.vy = 0.0
        
    def read_imu(self):
        
        self.imu_socket.flushInput()
        self.imu_socket.flushInput()
        self.imu_socket.flush()
        current_raw_imu = self.imu_socket.readline()
	
        try:
            current_imu = map(float,current_raw_imu.split(" "))
            pitch, yaw, roll = current_imu
            
            return pitch, yaw, roll
            
        except:
            print "Failed to parse:", imu_data
            import traceback
            traceback.print_exc()
                
            return None
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.last_time = self.current_time
            self.current_time = rospy.get_rostime();
            
            print "Reading imu"
            _, self.th, _ = self.read_imu()
            
            
            
            
            dt = 0.1# (current_time - last_time)
            delta_x = (self.vx * np.cos(self.th) - self.vy * np.sin(self.th))* dt
            delta_y = (self.vx * np.sin(self.th) - self.vy * np.cos(self.th))* dt
        
            self.x += delta_x;
            self.y += delta_y;
            
            odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)
   
            
            self.odom_broadcaster.sendTransform((self.x, self.y, 0.0),
                odom_quat, self.current_time, "base_link", "odom")
                
            rate.sleep()

if __name__ == '__main__':
    try:
		
		odo = Odometry()
		odo.run()
    except rospy.ROSInterruptException:
        pass
