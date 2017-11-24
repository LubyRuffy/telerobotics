#!/usr/bin/python

from mpu6050 import mpu6050
import serial
import numpy as np
import time
#import matplotlib.pyplot as plt
from mpu6050 import mpu6050
import math as m
import wiringpi as wp
from multiprocessing import Process, Value

r_now=Value('d',0.0)				# State variables shared between different threads
p_now=Value('d',0.0)
ls=Array('d',range(5))


mpu=mpu6050(0x68)

pinEngine=11
pinTurning=12 					# Motor PWM pins


def ser_readData():
	ser=serial.Serial('/dev/ttyS0',9600,timeout=1)
	index=[]
	val=[]
	flag=0
	t=time.time()
	x=0
	while x<4:
		data=""
		while data=="":
			data=ser.readline()
			words=data.split(",")
			temp_index=int(words[0])
			temp_val=float(words[1])
		#	print temp_index
		#	if (temp_index==0 or flag>0):
			index=index.append(temp_index)
			val=val.append(temp_val)
		#		flag=1
	return index,val,time.time()-t

def read_imuData():
	LPF_const=0.01					#Filter constant accel low pass filter
	Gpx=0						# Accel data in terms of G-Force
	Gpy=0
	Gpz=0
	r_past=0					#Previous roll angle
	p_past=0					#Previous pitch angle
	acc_past=0					#Previous accel magnitude
	Ts=0						#(NEEDS UPDATE!!!)Sample rate of the sensor 

	#global mpu,LPF_const,Gpx,Gpy,Gpz,r_past,p_past,acc_past
	while 1:
		t=time.time()
		accel_data=mpu.get_accel_data()			#Data acquisition in G-Force
		gyro_data=mpu.get_gyro_data()

		# Discrete time Low Pass Filter for accelerometer data
		Gpx=LPF_const*accel_data['x']+(1-LPF_const)*Gpx
		Gpy=LPF_const*accel_data['y']+(1-LPF_const)*Gpy
		Gpz=LPF_const*accel_data['z']+(1-LPF_const)*Gpz

		#Convert Accel data into Roll and pitch angles
		acc=np.sqrt(Gpx**2+Gpy**2+Gpz**2)
		r_acc=np.rad2deg(m.atan2(Gpy,Gpz))
		p_acc=np.rad2deg(m.atan2(-Gpx,np.sqrt(Gpy**2+Gpz**2)))
		diff=acc-acc_past
		acc_past=acc

		# Convert gyro data to roll,pitch and yaw angles
		r_gyr=r_past+(gyro_data['x']*Ts)
		p_gyr=p_past+(gyro_data['y']*Ts)

		sig=0.4								#Normal Distribution parameters
		mu=1
		alpha=(1/(sig*(np.sqrt(2*np.pi))))*(np.exp(-1*((acc-mu)**2)/(2*(sig**2))))

		r_now.value.=(r_gyr*alpha)+(r_acc*(1-alpha))
		p_now.value=(p_gyr*alpha)+(p_acc*(1-alpha))

		r_past=r_now.value
		p_past=p_now.value
		Ts=time.time()-t

		print "r_acc=",r_acc
		print "p_acc=",p_acc

		print "roll=",r_now
		print "pitch=",p_now

# Setup function. PWM duty cycle is between 0 and 100, so 50 is standstill

def setupMotors(pEngine,pTurning):
	wp.wiringPiSetup()
	pin1=pEngine
	pin2=pTurning
	wp.pinMode(pinEngine,wp.OUTPUT)
	wp.pinMode(pinTurning,wp.OUTPUT)
	wp.softPwmCreate(pinEngine,0,100)
    	wp.softPwmCreate(pinTurning,0,100)
    	wp.softPwmWrite(pinEngine,50)
    	wp.softPwmWrite(pinTurning,50)

# Sets the engine speed. Between -100 and 100, negative values reverse.
def setSpeed(speed):
    	wp.softPwmWrite(pinEngine,round((speed+100)/2))

# Sets the turning rate. Between -100 and 100,  Positive is right, negative is left
def turn(val)
    	wp.softPwmWrite(pinEngine,round((val+100)/2))
	
def tof_loop():
        ser=serial.Serial('/dev/ttyS0',9600,timeout=1)
        while 1:
                while data=="":
                        data=ser.readline()
                words=data.split(",")
                index=int(words[0])
                val=float(words[1])
                ls[index]=val
                data=""



if __name__=='__main__':
#	ser_init()
#	print ser_readData()
#	while 1:
#		read_imuData()

	tof=Process(name='TOF_Loop',target=tof_loop)
	imu=Process(name='IMU',target=read_imuData)
	
	tof.daemon=True
	imu.daemon=True
	
	tof.start()
	imu.start()
	

