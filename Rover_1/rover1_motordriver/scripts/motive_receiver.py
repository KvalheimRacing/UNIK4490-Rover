#!/usr/bin/env python

import json
import socket
import struct

import rospy
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import * 

#Static multicast address
MCAST_GRP = '239.255.42.199'
#Port to listen on
MCAST_PORT = 1600

class GPSReceiver(object):
    def __init__(self):	
        rospy.init_node('GPSReceiver', anonymous=True)
		
       # self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.gps_pub = rospy.Publisher("gps", Pose, 0)
        
        self.current_time = rospy.get_rostime();
        
        #Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                                     # to MCAST_GRP, not all groups on MCAST_PORT
        #Join multicast group
        mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
    def run(self):
        rover_id = None
        
        while not rospy.is_shutdown():
            #Retrieve data, 10240 is slightly random, messages might be
            #smaller or larger
            data = self.sock.recv(10240)
            try:
                #Convert to JSON data
                filtered = json.loads(data)
                #Filtered now contains JSON data which can be used for
                #GPS like applications or other use-cases
                if rover_id is None:
                    keys = filtered.keys()
                    rover_id = keys[0]
                            
                msg = Pose()
                msg.position.x = filtered[rover_id]["pos"]['x']
                msg.position.y = filtered[rover_id]["pos"]['y']
                msg.position.z = filtered[rover_id]["pos"]['z']
                
                print msg.position.x, msg.position.y, msg.position.z
                
                self.gps_pub.publish(msg)
                
            except ValueError:
                #Did not find myself
                continue
            except KeyError:
                print "Received JSON data which did not contain 'pos', 'vel',\
                    'pos_err' or 'vel_err'"
                print data
                
            #rospy.spinOnce()
        
        #Close socket like a good citizen
        self.sock.close()

if __name__ == '__main__':
    try:
		gps = GPSReceiver()
		gps.run()
    except rospy.ROSInterruptException:
        pass
