#!/usr/bin/python

from mpu6050 import mpu6050
import serial
import numpy as np
import time
import matplotlib.pyplot as plt
import math as m

acc_x=0
acc_y=0
acc_z=0

mpu=mpu6050(0x68)

t=time.time()


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


def imu_readdata():
	#Insert from Rpi
	# Convert accel data to roll,pitch and yaw angles
	acc=np.sqrt(Gpx**2+Gpy**2+Gpz**2)
	r_acc=np.rad2deg(m.atan2(Gpy,Gpz))
	p_acc=np.rad2deg(m.atan2(-Gpx,np.sqrt(Gpy**2+Gpz**2)))
	diff=acc-acc_past
	acc_past=acc
	# Convert gyro data to roll,pitch and yaw angles
	r_gyr=r_past+(x_gyr*Ts)
	p_gy=p_past+(y_gyr*Ts)
	
	sig=0.4								#Normal Distribution parameters
	mu=1
	alpha=(1/(sig*(np.sqrt(2*np.pi))))*(np.exp(-1*((acc-mu)**2)/(2*(sig**2))))
	
	r_now=(r_gyr*alpha)+(r_acc*(1-alpha))
	p_now=(p_gyr*alpha)+(p_acc*(1-alpha))
	
if __name__=='__main__':
#	ser_init()
	print ser_readData()

