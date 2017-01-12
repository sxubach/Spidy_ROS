
#!/usr/bin/python

import time
import numpy
import socket
from thread import *
import threading

#Socket address
HOST = '127.0.0.1'
PORT = 8888

OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
ENDC = '\033[0m'

distance_U = 42
accel_X = 1
accel_Y = 2
accel_Z = 3
gyro_X = 3
gyro_Y = 2
gyro_Z = 1

## Socket configuration start
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'

s.bind((HOST, PORT))
print 'Socket bind succesful'

s.listen(10) #number of tolerable unaccepted connections

def client_thread(c):
        print OKBLUE + 'Client connected' + ENDC
        while(1):
		print ' '
	        print OKBLUE + 'Trhead execution count' + ENDC
		
		state = int(c.recv(1024))
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
			pwm_value = int(c.recv(1024))
			print OKBLUE + 'pwm[' + str(x) + '] = '+ str(pwm_value) + ENDC
        			
		c.send("done")
        c.close()


while(1):
        c, addr = s.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1]) 
       
	t1 = threading.Thread(target=client_thread, args=((c,)))
	t1.daemon = True
	t1.start()

	while(t1.isAlive()):		
		start = time.time()

		distance_U = 42
		accel_X = 1
		accel_Y = 2
		accel_Z = 3
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
		
		end = time.time()
		#print str(end) + ' - ' + str(start) + ' = ' + str(end-start)
		time.sleep(0.01)

s.close()


