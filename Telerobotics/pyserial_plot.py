#!/usr/bin/python

import serial
import numpy as np
import matplotlib.pyplot as plt
import time

t=time.time() 
ser=serial.Serial('/dev/ttyS0',9600,timeout=1)

data=""

while (time.time()-t)<10:
	while data=="":
		data=ser.readline()	
	words=data.split(",")
	index=int(words[0])
	val=float(words[1])
	data=""

	print index*2
	print val
	plt.plot(val*np.sin(index*np.pi/4),val*np.cos(index*np.pi/4))
	plt.hold()
plt.show()
plt.savefig()





