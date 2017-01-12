
#!/usr/bin/python

import time
import numpy
import socket
import smbus
from thread import *
import threading

#Socket address
HOST = '192.168.0.55'
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
pwm=[60,90,60,5, 45,30,45,20, 60,0,85,60]

#i2c addresses
arduino_address = 0x04
accel_address = 0x68

def read_word(address, register):
	high = bus.read_byte_data(address, register)
	low = bus.read_byte_data(address, register+1)
	val = (high << 8) + low

	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val


def init_accelerometer():
	bus.write_byte_data(accel_address, 0x6B, 0) #wake up sensor - power management

def try_io(call, tries=10):
    assert tries > 0
    error = None
    result = None

    while tries:
        try:
            result = call()
        except IOError as e:
            error = e
            tries -= 1
        else:
            break

    if not tries:
        raise error

    return result


## I2C configuration start
# for RPI version 1, use "bus = smbus.SMBus(0)"
print 'Configuring i2c ...'
bus = smbus.SMBus(1)

init_accelerometer()
print 'Done!'
## I2C configuration end


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
	        print OKBLUE + 'Trhead execution' + ENDC
	
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

		for i in range(0, 12):
			c.send("ack.")
			pwm[i] = int(c.recv(1024))
		c.send("done")	
        c.close()

while(1):
        c, addr = s.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        
	t1 = threading.Thread(target=client_thread, args=((c,)))
	t1.daemon = True
	t1.start()

	while(t1.isAlive()):	

		#print 'Reading from Arduino ...'
		#print 'Ultrasound'
		bus.write_byte(arduino_address, 100)

		low = bus.read_byte(arduino_address)
		high = bus.read_byte(arduino_address)
		distance_U = (high << 8) | low
	
		#print 'Accelerometer'
		accel_X = read_word(accel_address, 0x3b)
		accel_Y = read_word(accel_address, 0x3d)
		accel_Z = read_word(accel_address, 0x3f)

		#print 'Gyroscope'
		gyro_X = read_word(accel_address, 0x43)
		gyro_Y = read_word(accel_address, 0x45)
		gyro_Z = read_word(accel_address, 0x47)

		for i in range(0, 12):
			try_io(lambda: bus.write_byte(arduino_address, i))
			try_io(lambda: bus.write_byte(arduino_address, pwm[i]))
			
			print 'pwm[' + str(i) + '] = '+ str(pwm[i])
	
		print 'distance_U =' + str(distance_U)
		print 'accel_X =' + str(accel_X)
		print 'accel_Y =' + str(accel_Y)
		print 'accel_Z =' + str(accel_Z)
		print 'gyro_X =' + str(gyro_X)
		print 'gyro_Y =' + str(gyro_Y)
		print 'gyro_Z =' + str(gyro_Z)
		print " "

		time.sleep(0.1)
s.close()

GPIO.cleanup()
