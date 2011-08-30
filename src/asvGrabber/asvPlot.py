#!/usr/bin/env python

import sys
#import numpy as np
from numpy import *
import matplotlib as mplot
#import mplot.pyplot as pplot
#import matplotlib.pyplot as plt
import pylab as p
#from mpl_toolkits.axes_grid1 import host_subplot

#
def dec2hex(n):
	#
	"""return the hexadecimal string representation of integer n"""
	#
	return "%X" % n

 
#
def hex2dec(s):
	#
	"""return the integer value of a hexadecimal string s"""
	#
	return int(s, 16)

#initilization
xMask  = hex2dec("FF")
yMask  = hex2dec("7F00")
xShift = 0
yShift = 8
 
#execution
data = sys.stdin.readlines()
size = len(data)
x = zeros(size, int)
y = zeros(size, int)
print "Counted", len(data), "lines."


for i in range (0,size):
	data2 = data[i].rsplit("\n")
	#print "data2: \n", data2
	dataS = data2[0].split(' ')
	#print "dataS1: ", hex2dec(dataS[0]) #address
	#print "dataS2: ", hex2dec(dataS[1]) #timestamp
	#print "dataS: \n", dataS
	address   = hex2dec(dataS[0])
	timestamp = hex2dec(dataS[1])
	x[i] = (address & xMask) 
	x[i] = x[i] >> xShift
	y[i] = (address & yMask)
	y[i] = y[i] >> yShift
	#print "x:", x[i], "y:", y[i]

#x = array( [0,1,2,3,4,5,6,7,8,9])
#y = array( [0,1,2,3,4,5,6,7,8,9])
#s = "BAD"
#print "BAD>", hex2dec(s), "!"

print x
print y

#ax = host_subplot(111, axes_class=AA.Axes)#
#fig = plt.figure()
#ax = fig.add_subplot(111)

p.plot(x,y,'o')
#pylab.show()
#plt.draw()
p.show()
