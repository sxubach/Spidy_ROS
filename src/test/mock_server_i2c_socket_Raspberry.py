
#!/usr/bin/python

import time
import numpy
import socket
from thread import *

#Socket address
HOST = '127.0.0.1'
PORT = 8888

OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
ENDC = '\033[0m'

## Socket configuration start
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'

s.bind((HOST, PORT))
print 'Socket bind succesful'

s.listen(10) #number of tolerable unaccepted connections
'''
c, addr = s.accept()
print 'Connected with ' + addr[0] + ':' + str(addr[1])
## Socket configuration end


## Main loop
print ' '
while(1):
	print 'Setting values ' + str(count) + ':'
	count = count + 1

	try:	
		#print 'Ultrasound start'
		distance_U = 42
		#print 'distance_U =' + str(distance_U)

		#print 'Accelerometer'
		accel_X = 1
		accel_Y = 2
		accel_Z = 3

		#print 'Gyroscope'
		gyro_X = 3
		gyro_Y = 2
		gyro_Z = 1

		print 'distance_U =' + str(distance_U)
		print 'accel_X =' + str(accel_X)
		print 'accel_Y =' + str(accel_Y)
		print 'accel_Z =' + str(accel_Z)
		print 'gyro_X =' + str(gyro_X)
		print 'gyro_Y =' + str(gyro_Y)
		print 'gyro_Z =' + str(gyro_Z)
		print " "
		
		state = ord(c.recv(1024))-10
		c.send(str(distance_U))
		c.recv(1024)
		c.send(str(accel_X))
		c.recv(1024)
		c.send(str(accel_Y))
		c.recv(1024)
		c.send(str(accel_Z))
		c.recv(1024)
		c.send(str(gyro_X))
		c.recv(1024)
		c.send(str(gyro_Y))
		c.recv(1024)
		c.send(str(gyro_Z))
		c.recv(1024)

		for x in range(0, 12):
			c.send("ack.")
			pwm_value = ord(c.recv(1024))-10
			print 'pwm[' + str(x) + '] = '+ str(pwm_value)
			

	except:
		print 'Connection lost'
		c.close()
		s.listen(10)
		c, addr = s.accept()
		print 'after accept'
	
	print " "
	print " "

c.close()
s.close()
'''

def client_thread(c):
        print OKBLUE + 'Client connected' + ENDC
        count = 0
        while(1):
	        print OKBLUE + 'Trhead count = ' + str(count) + ENDC
                count = count +1

		distance_U = 42
		accel_X = 1
		accel_Y = 2
		accel_Z = 3
		gyro_X = 3
		gyro_Y = 2
		gyro_Z = 1

		print OKBLUE + 'distance_U =' + str(distance_U) + ENDC
		print OKBLUE + 'accel_X =' + str(accel_X) + ENDC
		print OKBLUE + 'accel_Y =' + str(accel_Y) + ENDC
		print OKBLUE + 'accel_Z =' + str(accel_Z) + ENDC
		print OKBLUE + 'gyro_X =' + str(gyro_X) + ENDC
		print OKBLUE + 'gyro_Y =' + str(gyro_Y) + ENDC
		print OKBLUE + 'gyro_Z =' + str(gyro_Z) + ENDC
		print " "
		
		state = ord(c.recv(1024))-10
		c.send(str(distance_U))
		c.recv(1024)
		c.send(str(accel_X))
		c.recv(1024)
		c.send(str(accel_Y))
		c.recv(1024)
		c.send(str(accel_Z))
		c.recv(1024)
		c.send(str(gyro_X))
		c.recv(1024)
		c.send(str(gyro_Y))
		c.recv(1024)
		c.send(str(gyro_Z))
		c.recv(1024)

		for x in range(0, 12):
			c.send("ack.")
			pwm_value = ord(c.recv(1024))-10
			print OKBLUE + 'pwm[' + str(x) + '] = '+ str(pwm_value) + ENDC
        
        c.close()


while(1):
        c, addr = s.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        
        start_new_thread(client_thread,(c,))

s.close()


