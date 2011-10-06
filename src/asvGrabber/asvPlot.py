#!/usr/bin/python 

import sys
import getopt,string 
#import numpy as np
from numpy import *
import matplotlib as mplot
#import mplot.pyplot as pplot
#import matplotlib.pyplot as plt
import pylab as p
#from mpl_toolkits.axes_grid1 import host_subplot

#/usr/bin/env python

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

#
def int2bin(n, count=24):
	#
	"""returns the binary of integer n, using count number of digits"""
	#
	return "".join([str((n >> y) & 1) for y in range(count-1, -1, -1)])

def flipBits(n, count = 24):
	out = 0
	for  i in range (count - 1 ,-1, -1):
		mask = 1;
		maskout = 1;		
		for j in range( 0, i): 
			mask = mask * 2;
			
		for j in range (0, count - 1 - i):
	       		maskout = maskout * 2;
				
	       	value = mask & n
	       	if value:
	       		bitvalue = 1	    
	       	else: 
	       		bitvalue = 0			
	       	out = out + bitvalue * maskout;
	return out

#reading parameters
print sys.argv # returns: ['param.py']
try:
    options, xarguments = getopt.getopt(sys.argv[1:],
    'ha', ['asv', 'aex', 'version'])
except getopt.error:
    print "Error: You tried to use an unknown option Try -h"
    sys.exit(0)

#opts, extraparams = getopt.getopt(sys.argv[1:],'ha', ['--list', '--view', 'version']) 
# starts at the second element of argv since the first one is the script name
# extraparms are extra arguments passed after all option/keywords are assigned
# opts is a list containing the pair "option"/"value"
#print 'Opts:',opts
#print 'Extra parameters:',extraparam

for a in options[:]:
    if a[0] == "--asv": 
        print "ASV mode active"
        xMask   = hex2dec("FF")
	yMask   = hex2dec("7F00")
	xShift  = 0
	yShift  = 8
	xoffset = -112
	yoffset = -56
	reverseBits = 1
    else:
        print "AEX mode active"
        xMask   = hex2dec("FE")
	yMask   = hex2dec("7F00")
	xShift  = 1
	yShift  = 8
	xoffset = 0
	yoffset = 0
	reverseBits = 0
 
#initilization


 
#execution
data = sys.stdin.readlines()
size = len(data)
x = zeros(size, int)
y = zeros(size, int)
print "Counted", len(data), "lines."
err_address        = 0
overflow_address   = 0
first_wrong        = 0
previous_timestamp = 0;
previous_address   = 0;
timestamp          = 0;
address            = 0;
diff               = 0;
lastTimestamp      = 0

for i in range (0,size):
	data2 = data[i].rsplit("\n")
	#print "data2: \n", data2
	dataS = data2[0].split(' ')
	#print "dataS1: ", hex2dec(dataS[0]) #address
	#print "dataS2: ", hex2dec(dataS[1]) #timestamp
	#print "dataS: \n", dataS
	previous_address = address;
	address = hex2dec(dataS[0])

	previous_timestamp = timestamp;
	timestamp = hex2dec(dataS[1])
	
	#print "reversing 10", flipBits(10,4)
	
	if i==0 :
		print "FIRST timestamp:" ,timestamp ;
		lastTimestamp = timestamp
	#print "address:" , int2bin(address,32);


	#if address < 14448:
	#	err_address = err_address + 1
		#x[i] = 255;
		#y[i] = 255;

	#if address > 65535 and not(first_wrong):
	#        overflow_address = overflow_address + 1
	#	print "block:",dataS[1], ":",previous_timestamp, " diff:",(previous_timestamp - lastTimestamp) * 160
	#	lastTimestmap = previous_timestamp;
	#	first_wrong = 1
		#x[i] = 255;
		#y[i] = 255;

	#if address <= 65535 and previous_address <= 65535:
	#	first_wrong = 0

	# enable if when filtering of correctness is active
        #if address > 14448 and address < 65535:
	x[i] = (address & xMask) 
	x[i] = x[i] >> xShift
	y[i] = (address & yMask)
	y[i] = y[i] >> yShift

	if a[0] == "--asv": 
		yflip = flipBits(y[i],7)
	else:
		yflip = y[i]
	
	y[i] = yflip + yoffset
	        #print "yFlip:", yflip, "y:", y[i]
	#y[i] = y[i] + offsety	
	x[i] = x[i]  + xoffset

#x = array( [0,1,2,3,4,5,6,7,8,9])
#y = array( [0,1,2,3,4,5,6,7,8,9])
#s = "BAD"
#print "BAD>", hex2dec(s), "!"

print x
print y
print "WARNING: number of events  smaller than 3870", err_address
print "WARNING: number of address greater than FFFF", overflow_address

#ax = host_subplot(111, axes_class=AA.Axes)#
#fig = plt.figure()
#ax = fig.add_subplot(111)

p.plot(x,y,'o')
#pylab.show()
#plt.draw()
p.show()
