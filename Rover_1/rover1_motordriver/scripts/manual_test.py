import serial


from time import sleep, clock
import numpy as np

FORWARD = -1
BACKWARD = 1

WHEELS_RIGHT = 2
WHEELS_LEFT = 1

# motor skal vaere enten WHEELS_LEFT eller WHEELS_RIGHT
def set_speed(serial_socket, motor, speed):
	# sets speed to max 100 or min -100
	speed = min(100,max(-100,-speed))
	
	if motor == 2:
		offset = 127
	else:
		offset = 0
		
	b = int(speed/100.0 * 62)+offset+63
	#print motor, speed, b
	serial_socket.write(bytes(chr(b)))

if __name__=="__main__":
	
	s =  serial.Serial(port='/dev/ttyUSB1', baudrate=9600)
	encoder_socket =  serial.Serial(port='/dev/ttyUSB2', baudrate=9600, timeout=0.1)
		
	encoder_socket.readline()
	s.write(bytes(chr(0)))
	sleep(1)
	
	
	last_encoders = None
	last_time = None
	pid_acc = np.zeros(4)
	last_effect = np.zeros(4)
	for x in xrange(10000):
		set_point = np.array([-1100.0,0.0,0.0,0.0])
		
		encoder_data = encoder_socket.readlines()
		#print encoder_data
		if encoder_data is not None and len(encoder_data) >0:
			current_time = clock()
			
			current_encoders = np.zeros(4)
			count = 0
			for x in xrange(len(encoder_data)-1, -1, -1):
				line = encoder_data[x]
				try:
					line = line.split(":")[:-1]
					#print len(line)
					if len(line) < 4:
						continue
					else:
						current_encoders += np.array(map(int, line))
						count += 1
					#print line
					#print line
				except:
					continue
			current_encoders /= float(count)
				
			if last_time is not None and current_encoders is not None:
				delta = current_encoders - last_encoders
				delta_time = current_time - last_time
				
				
				error = current_encoders-set_point
				last_error = last_encoders - set_point
				delta_error = error-last_error
				pid_acc += error*delta_time
				kp = 0.005
				kd = 0.0
				ki = 0.001
				effect = kp*error[0] + kd * delta_error[0]/delta_time + ki* pid_acc[0]
				
				
				print "Set point:", set_point
				print "Current encoders:", current_encoders
				print "Error:", error
				print "Effect:" , effect, pid_acc[0]
				if last_effect[0]+effect > 0:
				
					print "Going forward"
					set_speed(s,1, FORWARD, min(100,max(0,int(last_effect[0]+effect ))))
				else:
					print "Going backward"
					set_speed(s,1, BACKWARD, min(100,max(0,-int(last_effect[0]+effect ))))
				speed = delta / delta_time
				print speed[0], current_encoders[0],  encoder_data
				last_effect[0] = last_effect[0]+effect
			if current_encoders is not None:
				


				last_encoders = current_encoders
				last_time = current_time
				
				print current_encoders
		sleep(0.2)
	motor = 1
	#while True:
		#for motor in [1,2]:
			#for direction in [FORWARD,BACKWARD]:
				
				#for speed in xrange(0,100):
					#set_speed(s,motor, direction, speed)
					#sleep(0.05)
				#sleep(2)
				#for speed in xrange(100, -1,-1):
					#set_speed(s,motor, direction, speed)
					#sleep(0.05)
		
			

import serial


from time import sleep, clock
import numpy as np

FORWARD = 1
BACKWARD = -1


if __name__=="__main__":
	
	s =  serial.Serial(port='/dev/ttyUSB1', baudrate=9600)
	encoder_socket =  serial.Serial(port='/dev/ttyUSB2', baudrate=9600, timeout=0.1)
		
	encoder_socket.readline()
	s.write(bytes(chr(0)))
	sleep(1)
	
	
	last_encoders = None
	last_time = None
	pid_acc = np.zeros(4)
	last_effect = np.zeros(4)
	for x in xrange(10000):
		set_point = np.array([-100.0,0.0,0.0,0.0])
		
		encoder_data = encoder_socket.readlines()
		#print encoder_data
		if encoder_data is not None and len(encoder_data) >0:
			current_time = clock()
			
			current_encoders = None
			for x in xrange(len(encoder_data)-1, -1, -1):
				line = encoder_data[x]
				try:
					line = line.split(":")[:-1]
					#print len(line)
					if len(line) < 4:
						continue
					else:
						current_encoders = np.array(map(int, line))
						break
					#print line
					#print line
				except:
					continue
				
				
			if last_time is not None and current_encoders is not None:
				delta = current_encoders - last_encoders
				delta_time = current_time - last_time
				
				
				error = current_encoders-set_point
				last_error = last_encoders - set_point
				delta_error = error-last_error
				pid_acc += error*delta_time
				kp = 0.01
				kd = 0.0
				ki = 0.0
				effect = kp*error[0] + kd * delta_error[0]/delta_time + ki* pid_acc[0]
				
				
				print "Set point:", set_point
				print "Current encoders:", current_encoders
				print "Error:", error
				print "Effect:" , effect, pid_acc[0]
				if last_effect[0]+effect > 0:
				
					print "Going forward"
					set_speed(s,1, FORWARD, min(100,max(0,int(last_effect[0]+effect ))))
				else:
					print "Going backward"
					set_speed(s,1, BACKWARD, min(100,max(0,-int(last_effect[0]+effect ))))
				speed = delta / delta_time
				print speed[0], current_encoders[0],  encoder_data
				last_effect[0] = last_effect[0]+effect
			if current_encoders is not None:
				


				last_encoders = current_encoders
				last_time = current_time
				
				print current_encoders
		sleep(0.2)
	motor = 1
	#while True:
		#for motor in [1,2]:
			#for direction in [FORWARD,BACKWARD]:
				
				#for speed in xrange(0,100):
					#set_speed(s,motor, direction, speed)
					#sleep(0.05)
				#sleep(2)
				#for speed in xrange(100, -1,-1):
					#set_speed(s,motor, direction, speed)
					#sleep(0.05)
		
			

