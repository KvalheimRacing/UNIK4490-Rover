#!/usr/bin/env python
import rospy
import numpy as np
import serial

from geometry_msgs.msg import Twist
from manual_test import set_speed, FORWARD, BACKWARD, WHEELS_RIGHT, WHEELS_LEFT
from time import sleep
from pid import PidControl
import rover_motordriver.msg as msgs

ENCODER_TIMEDELTA = 0.1
ENCODER_STEPPERREV = 100
ENCODER_REVSPERWHEELROT = 300 #1200
ENCODER_WHEELCIRUMFERENCE = 4.75*(2.54*3.14*2.0)
VELOCITY_CONVERTIONCONST =  ENCODER_WHEELCIRUMFERENCE/(ENCODER_TIMEDELTA * ENCODER_STEPPERREV * ENCODER_REVSPERWHEELROT)
ENCODER_NORM = np.array([1,1,-1,-1])

PID_CONTROL_RATE = 5

DISTANCE_THRESHOLD = 1.0
ANGLE_THRESHOLD = 5.0

	
def read_encoder(e_socket):
	encoder_data = e_socket.readlines()
	
	parsed_data = []
	for i in xrange(len(encoder_data)):
		try:
			current_encoders = encoder_data[i]
			current_encoders = map(int,current_encoders.split(":"))
			velocities = np.array(current_encoders[:4])
			abs_positions = np.array(current_encoders[4:])
			
			velocities = np.multiply(velocities * VELOCITY_CONVERTIONCONST, ENCODER_NORM)
			abs_positions = np.multiply(abs_positions,ENCODER_NORM)
            velocities_ticks = np.multiply(velocities_ticks, ENCODER_NORM)
			
			parsed_data.append((velocities, abs_positions))
		except:
			print "Failed to parse:", encoder_data[i]
			import traceback
			traceback.print_exc()
			continue
	return parsed_data
	
def calibrate(s_socket, e_socket):
	
	encoder_speeds = []
	for speed in xrange(0,100,5):
		set_speed(s_socket, 1,  speed)
		set_speed(s_socket, 2,  speed)
		
		e_socket.readlines()#Trash all current samples
		sleep(1)
		
		try:
			velocities, abs_positions = read_encoder(e_socket)[-1]
			print velocities, abs_positions
		except:
			import traceback
			traceback.print_exc()
			velocities = None
		encoder_speeds.append(velocities)
		if rospy.is_shutdown():
			break
			
	def plot(encoder_speeds):
		import matplotlib.pyplot as plt
		plt.figure()
		plt.axis([0,100,0,30])
		for series in xrange(4):
			x = np.arange(0,100,5)
			y = map(lambda t: t[series], encoder_speeds)
			plt.plot(x,y)
			
		plt.show()
	
	set_speed(s_socket, 1,  0)
	set_speed(s_socket, 2,  0)
	
	#plot(encoder_speeds)
	
	from scipy import stats
	x = map(lambda t: t[0], encoder_speeds)
	y = np.arange(0,100,5)
	slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)

	print "a:",slope, "b:",intercept, "(ax+b=y)"
	print 'r value', r_value
	print  'p_value', p_value
	print 'standard deviation', std_err
	
	return (slope, intercept)
def test_response(s_socket, e_socket):
	
	speed = 0
	set_speed(s_socket, 1, FORWARD, speed)
	set_speed(s_socket, 2, FORWARD, speed)
		
	e_socket.readlines()#Trash all current samples
	sleep(0.5)
	speed = 80
	set_speed(s_socket, 1, FORWARD, speed)
	set_speed(s_socket, 2, FORWARD, speed)
		
	sleep(1.5)
		
	velocities = map(lambda t: t[0], read_encoder(e_socket))
	
	
	for v in velocities:
		print v
		
	def plot(encoder_speeds):
		import matplotlib.pyplot as plt
		plt.figure()
		num_samples = len(encoder_speeds)
		plt.axis([0,num_samples*0.1,0,30])
		x = np.arange(0,num_samples*0.1,0.1)
		for series in xrange(4):
			y = map(lambda t: t[series], encoder_speeds)
			plt.plot(x,y)
			
		plt.show()
	
	set_speed(s_socket, 1, FORWARD, 0)
	set_speed(s_socket, 2, FORWARD, 0)
	
	plot(velocities)
	
	
	
class Autopilot(object):
    def velocity_handler(self,message):
        #print message
        self.reset()
        self.velocity_setpoint = np.array([message.left_vel, message.right_vel])
        self.velocity_duration = None if message.duration < 0.01 else message.duration
        
    def distance_handler(self,message):
        #print message
        self.reset()
        self.distance_setpoint = message.distance
        
    def angle_handler(self,message):
        #print message
        self.reset()
        self.angle_setpoint = message.angle
        
    def position_handler(self,message):
        #print message
        self.reset()
        self.position_setpoint = np.array([message.position.x, message.position.y])
        
    def reset(self):
        self.velocity_setpoint = None
        self.distance_setpoint = None
        self.angle_setpoint = None
        self.position_setpoint = None
        
    def __init__(self):
        rospy.init_node('autopilot', anonymous=True)
        
        rospy.Subscriber("/cmd_vel", msgs.Velocity , self.velocity_handler)
        rospy.Subscriber("/cmd_angle", msgs.Angle, self.angle_handler)
        rospy.Subscriber("/cmd_dist", msgs.Distance, self.distance_handler)
        rospy.Subscriber("/cmd_pos", msgs.Position, self.position_handler)

        self.pub_est_vel = rospy.Publisher("/est_vel", msgs.Velocity, queue_size=10)

        self.serial_socket =  serial.Serial(port='/dev/ttyUSB0', baudrate=9600)
        self.encoder_socket =  serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.01)
            
        self.rate = rospy.Rate(PID_CONTROL_RATE) 
        self.dt = 1.0/float(PID_CONTROL_RATE)
        
        self.vel_pid_left = PidControl(0.0,2.0,0.0)
        self.vel_pid_right = PidControl(0.0,2.0,0.0)
        
        self.pos_pid_distance = PidControl(1.0,0.01,0.0)
        self.pos_pid_angle = PidControl(1.0,0.1,0.0)

        self.velocity_setpoint = None #Typically touple (v1,v2), wanted speed on right and left wheels
        self.velocity_duration = None
        
        self.angle_setpoint = None #Typically goto THETA angle
        
        self.distance_setpoint = None#Typically X distance to move forward
        
        self.position_setpoint =  np.array([50,0]) #Typically (x,y) coordinate, goes in straight line
                
        self.max_velocity = 30
        #calibrate(self.serial_socket, self.encoder_socket)
    def invert(self,pwm):
        velocity = 0.89*pwm+1.74
        return velocity
        
    def translate_dist_sp(self,avg_velocity):
        self.distance_setpoint -= np.sum(avg_velocity)/2.0*self.dt
        
        forward_velocity = self.pos_pid_distance(-np.floor(self.distance_setpoint),0, self.dt)
        forward_velocity = max(-self.max_velocity, min(self.max_velocity, forward_velocity))
        
        return np.array([forward_velocity, forward_velocity]) 
        
    def translate_angle_sp(self,avg_velocity):
        def angle_to_distance(angle):
            return 2.0*3.14 * 15.0 /360.0 * angle
        def distance_to_angle(distance):
            return distance/(2.0*3.14 * 15.0) * 360.0
        
        diff_vel = avg_velocity[0]-avg_velocity[1]
        distance_turned = diff_vel*self.dt/2.0
        self.angle_setpoint -= distance_to_angle(distance_turned)
        print "Error", self.pos_pid_angle.previous_error
        angular_velocity = self.pos_pid_angle(np.round(self.angle_setpoint,0),0, self.dt)
        angular_velocity = max(-self.max_velocity, min(self.max_velocity, angular_velocity))
        
        return np.array([-angular_velocity, angular_velocity]) 
        
    def calc_throttle(self, avg_velocity):
        def helper(pid_ctrl, index):
            pid_velocity = pid_ctrl(avg_velocity[index],self.velocity_setpoint[index], self.dt)
            if self.velocity_setpoint[index] > 0:
                base_velocity = self.invert(self.velocity_setpoint[index])
            else:
                base_velocity = -self.invert(-self.velocity_setpoint[index])
                
            new_velocity = int(base_velocity+pid_velocity)
            C = ["Left", "Right"]
            print C[index],"velocity contribs:", "PID", pid_velocity, "Base", base_velocity
            
            return new_velocity
            
        new_velocity_left = helper(self.vel_pid_left, 0)
        new_velocity_right = helper(self.vel_pid_right, 1)
        
        return (new_velocity_left, new_velocity_right)
        
    def run(self):
        while not rospy.is_shutdown():
            try:
                velocities, abs_positions = read_encoder(self.encoder_socket)[-1]
                avg_velocity_left = (velocities[0]+velocities[1])/2.0
                avg_velocity_right = (velocities[2]+velocities[3])/2.0
                avg_velocity = np.array([avg_velocity_left, avg_velocity_right])
                
            except:
                #import traceback
               # traceback.print_exc()
                
                rospy.logfatal("Error failed to parse velocities (try reinserting encoder reader)")
                rospy.signal_shutdown("")
                continue
                
                
            msg = msgs.Velocity()
            #Converting to m/s from cm/s
            msg.left_vel = avg_velocity_left/100.0
            msg.right_vel = avg_velocity_right/100.0
            
            self.pub_est_vel.publish(msg)
        
                
            if self.position_setpoint is not None:
                try:
                    self.distance_setpoint = np.linalg.norm(self.position_setpoint)
                    self.angle_setpoint = np.arctan2(self.position_setpoint[1],self.position_setpoint[0])/(2.0*3.14) * 360
                    self.position_setpoint = None
                    print "Position setpoint translated"
                except:
                    import traceback
                    traceback.print_exc()
                    
            if self.distance_setpoint is not None:
                try:
                    self.velocity_setpoint = self.translate_dist_sp(avg_velocity)
                    
                    print "Distance setpoint", self.distance_setpoint
                    if np.abs(self.distance_setpoint) < DISTANCE_THRESHOLD:
                        self.distance_setpoint = None
                        self.velocity_setpoint = None
                except:
                    import traceback
                    traceback.print_exc()
                
            if self.angle_setpoint is not None:
                try:
                    self.velocity_setpoint = self.translate_angle_sp(avg_velocity)
                    
                    print "Angle setpoint", self.angle_setpoint
                    if np.abs(self.angle_setpoint) < ANGLE_THRESHOLD:
                        self.angle_setpoint = None
                        self.velocity_setpoint = None
                except:
                    import traceback
                    traceback.print_exc()
            
            if self.velocity_setpoint is not None:			
                try:
                    left_throttle, right_throttle = self.calc_throttle(avg_velocity)
                    if self.velocity_duration is not None:
                        self.velocity_duration -= self.dt
                        if self.velocity_duration < 0:
                            self.velocity_duration = None
                            self.velocity_setpoint = None
                        
                    
                    set_speed(self.serial_socket, WHEELS_LEFT, left_throttle)
                    set_speed(self.serial_socket, WHEELS_RIGHT, right_throttle)
                    print velocities
                except:
                    import traceback
                    traceback.print_exc()
            else:
                set_speed(self.serial_socket, WHEELS_LEFT, 0)
                set_speed(self.serial_socket, WHEELS_RIGHT, 0)
                
            self.rate.sleep()
            
        #Stop both "tracks"
        set_speed(self.serial_socket, WHEELS_LEFT, 0)
        set_speed(self.serial_socket, WHEELS_RIGHT, 0)
        
        

if __name__ == '__main__':
    try:
		
		ap = Autopilot()
		ap.run()
    except rospy.ROSInterruptException:
        pass
