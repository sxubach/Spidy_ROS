
#!/usr/bin/python

import time
import numpy
import socket
import smbus

#Socket address
HOST = '192.168.0.42'
PORT = 8888

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
c, addr = s.accept()
print 'Connected with ' + addr[0] + ':' + str(addr[1])
## Socket configuration end


state = 0
## Main loop
while(1):
	#print 'Reading from Arduino ...'
	# print 'Ultrasound start'
	bus.write_byte(arduino_address, 100)

	low = bus.read_byte(arduino_address)
	high = bus.read_byte(arduino_address)
	distance_U = (high << 8) | low

	print 'low =' + str(low)
	print 'high =' + str(high)
	print 'distance_U =' + str(distance_U)

	
	# print 'Accelerometer'
	accel_X = read_word(accel_address, 0x3b)
	accel_Y = read_word(accel_address, 0x3d)
	accel_Z = read_word(accel_address, 0x3f)

	# print 'Gyroscope'
	gyro_X = read_word(accel_address, 0x43)
	gyro_Y = read_word(accel_address, 0x45)
	gyro_Z = read_word(accel_address, 0x47)

	try:	
		if state==0:	
			state = ord(c.recv(1024))-10	#same protocol as i2c
		if state>=0 and state<=12:
			c.send("ack.")
			pwm_value = ord(c.recv(1024))-10

			bus.write_byte(arduino_address, state)
			bus.write_byte(arduino_address, pwm_value)
			state = 0;
		if state==100:
			c.send(str(distance_U))
			state = 0;
		if state==101:
			#print 'state 101'
			c.send(str(accel_X))
			#print 'accel_X sent'
			state = 0;
		if state==102:
			c.send(str(accel_Y))
			state = 0;
		if state==103:
			c.send(str(accel_Z))
			state = 0;
		if state==104:
			c.send(str(gyro_X))
			state = 0;
		if state==105:
			c.send(str(gyro_Y))
			state = 0;
		if state==106:
			c.send(str(gyro_Z))
			state = 0;

	except:
		s.listen(10)
		c, addr = s.accept()

c.close()
s.close()

GPIO.cleanup()
