#!/usr/bin/python

from mpu6050 import mpu6050
import serial
import numpy as np
import time
import matplotlib.pyplot as plt


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

if __name__=='__main__':
#	ser_init()
	print ser_readData()

