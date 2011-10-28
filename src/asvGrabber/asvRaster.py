#!/usr/bin/python 

import sys
import getopt,string 
#import numpy as np
from numpy import *
import matplotlib as mplot
from decimal import *
#import mplot.pyplot as pplot
#import matplotlib.pyplot as plt
import pylab as p
#import pylab as rasterPlot
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
    'ha', ['asv', 'aex','file', 'version', 'mmin=', 'mmax'])
except getopt.error:
    print "Error: You tried to use an unknown option Try -h"
    sys.exit(0)

#opts, extraparams = getopt.getopt(sys.argv[1:],'ha', ['--list', '--view', 'version']) 
# starts at the second element of argv since the first one is the script name
# extraparms are extra arguments passed after all option/keywords are assigned
# opts is a list containing the pair "option"/"value"
print 'Opts:',options
print 'Extra parameters:',xarguments

fileFlag = 0
for a in options[:]:
	
	if a[0] == "--file":
		print "reading from file"
		fileFlag = 1
	
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
		
	
#if a[0] == "--mmin":	
#`	print "min metapixel"
	
 
#initilization
metaPixelMax = 3
metaPixelMin = 0
timestampMax = 2000000
timestampMin = 0
sizeLUT = 10368
sizeYRaster = 144 * 72 * 12 * 2
sizeXRaster = 144 * 72 * 12 * 2

countEvent = 0

#data = sys.stdin.readlines()
# using the readfile function for data 
if fileFlag:
	#reading from buffer
	print "reading from file"
	execfile("./readFile.py")
	data = buffer
else:
	print "reading from stdin"
	data = sys.stdin.readlines()
	#size = len(data)
	#print "data length", size	

size = len(data)
print "data length", size
	

x = zeros(size, int)
y = zeros(size, int)
xRaster = zeros(sizeXRaster, int)
yRaster = zeros(sizeYRaster, int)
	
metax = zeros(sizeLUT, int)
metay = zeros(sizeLUT, int)
type  = zeros(sizeLUT, int)
polar = zeros(sizeLUT, int)


print "Counted", len(data), "lines."
err_address        = 0
overflow_address   = 0
first_wrong        = 0
previous_timestamp = 0
previous_address   = 0
timestamp          = 0
address            = 0
diff               = 0
lastTimestamp      = 0
firstTimestamp     = 0

#execution

for i in range (0,size):
	data2 = data[i].rsplit("\n")
	
	#print "data2[0]: \n", data2
	dataS = data2[0].split(' ')
	#print "dataS1: ", hex2dec(dataS[0]) #address
	#print "dataS2: ", hex2dec(dataS[1]) #timestamp
	#print "dataS: \n", dataS
	
	#print "dataS[0] \n", dataS[0]

	if dataS[0]=="#" :
		xchip = Decimal(dataS[1]);
		ychip = Decimal(dataS[2]);
		position = xchip * 72 + ychip
		
		#print "position", position
			
		metax[position] = Decimal(dataS[3])
		metay[position] = Decimal(dataS[4])
		polar[position] = Decimal(dataS[5])
		type [position] = Decimal(dataS[6])
	
		#print metax[position]," ", metay[position]," ", polar[position]," ", type[position], "\n"		
	else:	
		previous_address = address;
		address = hex2dec(dataS[0])

		previous_timestamp = timestamp;
		timestamp = hex2dec(dataS[1])
		if(timestamp - previous_timestamp > 38000):
			print "address1:" , dataS[0], timestamp -  previous_timestamp
		
	
		if countEvent==0 :
			print "FIRST timestamp:" ,timestamp ;
			firstTimestamp = timestamp
			lastTimestamp  = timestamp

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
		x[countEvent] = (address & xMask) 
		x[countEvent] = x[countEvent] >> xShift
	
		if(x[countEvent] == 0):
			print "address2:" , dataS[0], timestamp -  previous_timestamp	

		y[countEvent] = (address & yMask)
		y[countEvent] = y[countEvent] >> yShift

		if a[0] == "--asv": 
			yflip = flipBits(y[countEvent],7)
		else:
			yflip = y[countEvent]
	
		y[countEvent] = yflip + yoffset
	        #print "yFlip:", yflip, "y:", y[countEvent]
		#y[countEvent] = y[countEvent] + offsety	
		x[countEvent] = x[countEvent]  + xoffset
		
		position =  x[countEvent] * 72 + y[countEvent]

		if(position > 0 ):
			#print "x", x[countEvent],"y", y[countEvent], "position", position
			mx = metax[position] 
			my = metay[position] 
			po = polar[position] 
			ty = type [position] 		

			#print "mx", mx, "my", my, "po", po, "ty", ty
	
			yRaster[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
			xRaster[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			countEvent = countEvent + 1

#xRaster = array( [0,1,2,3,4,5,6,7,8,9])
#yRaster = array( [0,1,2,3,4,5,6,7,8,9])
#s = "BAD"
#print "BAD>", hex2dec(s), "!"

print xRaster
print yRaster
print "WARNING: number of events  smaller than 3870", err_address
print "WARNING: number of address greater than FFFF", overflow_address

#ax = host_subplot(111, axes_class=AA.Axes)#
#fig = plt.figure()
#ax = fig.add_subplot(111)

# plot of the events in Raster form
# extracting metapixels in required range

limitPixelMin = metaPixelMin * 12 * 2
limitPixelMax = metaPixelMax * 12 * 2
#timestampMin  = limitPixelMin
#timestampMax  = limitPixelMax 
#p.plot(xRaster[0:12],yRaster[0:12],'o',xRaster[12:24],yRaster[12:24],'ro',xRaster[24:50],yRaster[24:50],'mo')
p.plot(xRaster,yRaster,'o')

#for i, ax in enumerate(event.canvas.figure.axes): # Get the axes
#	ax.set_xlim(lims)
#ax.set_ylim()
#p.axis([timestampMin, timestampMax,metaPixelMin * 12 , (metaPixelMax + 1) * 12])
#pylab.show()
#plt.draw()
p.show()

# plot of the event in the Raster form 
#rasterPlot.plot(x[:], y[0:12],'o',x[], y[12:24],'ro',x, y[24:36],'mo')
#rasterPlot.show()

