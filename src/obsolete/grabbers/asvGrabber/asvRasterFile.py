#!/usr/bin/python 

import sys
import getopt,string 
from numpy import *
#import matplotlib as mplot
from decimal import *
from pylab import *

#
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

fileFlag = 1
asvFlag  = 1
aexFlag  = 0	

if asvFlag == 1: 
        print "ASV mode active"
        xMask   = hex2dec("FF")
	yMask   = hex2dec("7F00")
	xShift  = 0
	yShift  = 8
	xoffset = -112
	yoffset = -56
	reverseBits = 1
if aexFlag == 1:
        print "AEX mode active"
        xMask   = hex2dec("FE")
	yMask   = hex2dec("7F00")
	xShift  = 1
	yShift  = 8
	xoffset = 0
	yoffset = 0
	reverseBits = 0
	
 
#initilization
metaPixelMax = 3
metaPixelMin = 0
timestampMax = 2000000
timestampMin = 0
sizeLUT = 10368
#sizeYRaster = 144 * 72 * 12 * 2
#sizeXRaster = 144 * 72 * 12 * 2

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
xRaster = zeros(size, int)
yRaster = zeros(size, int)

xCdOff = zeros(size, int)
yCdOff = zeros(size, int)
xCdOn = zeros(size, int)
yCdOn = zeros(size, int)
xEM1L = zeros(size, int)
yEM1L = zeros(size, int)
xEM1H = zeros(size, int)
yEM1H = zeros(size, int)
xEM2L = zeros(size, int)
yEM2L = zeros(size, int)
xEM2H = zeros(size, int)
yEM2H = zeros(size, int)
xEM3L = zeros(size, int)
yEM3L = zeros(size, int)
xEM3H = zeros(size, int)
yEM3H = zeros(size, int)
xEM4L = zeros(size, int)
yEM4L = zeros(size, int)
xEM4H = zeros(size, int)
yEM4H = zeros(size, int)
xIFOn = zeros(size, int)
yIFOn = zeros(size, int)
xIFOff = zeros(size, int)
yIFOff = zeros(size, int)

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

		#if(timestamp - previous_timestamp > 38000):
			#print "address1:" , dataS[0], timestamp -  previous_timestamp
	
		if countEvent==0 :
			#print "FIRST timestamp:" ,timestamp ;
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
	
		#if(x[countEvent] == 0):
			#print "address2:" , dataS[0], timestamp -  previous_timestamp	

		y[countEvent] = (address & yMask)
		y[countEvent] = y[countEvent] >> yShift

		if asvFlag: 
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
			# divide the events in different types, but keep the same y coordinate
			if ty * 2 + po	 == 0:
				yCdOff[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xCdOff[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 1:
				yCdOn[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xCdOn[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 2:
				yEM1L[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM1L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 3:
				yEM1H[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM1H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 4:
				yEM2L[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM2L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 5:
				yEM2H[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM2H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty  * 2 + po== 6:
				yEM3L[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM3L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty * 2 + po == 7:
				yEM3H[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM3H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty  * 2 + po== 8:
				yEM4L[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM4L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty  * 2 + po== 9:
				yEM4H[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xEM4H[countEvent] = (timestamp - firstTimestamp) * 160 #ns\
			elif ty * 2 + po == 11:
				yIFOn[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xIFOn[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			elif ty  * 2 + po== 10:
				yIFOff[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
				xIFOff[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			#elif ty * 2 + po == 10:
			#	yIFOn[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
			#	xIFOn[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			#elif ty  * 2 + po== 11:
			#	yIFOff[countEvent] = (mx * 72 + my) * 12 + ty * 2 + po		
			#	xIFOff[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			countEvent = countEvent + 1

#xRaster = array( [0,1,2,3,4,5,6,7,8,9])
#yRaster = array( [0,1,2,3,4,5,6,7,8,9])
#s = "BAD"
#print "BAD>", hex2dec(s), "!"

# print for debug:
#print xRaster
#print yRaster
#print "WARNING: number of events  smaller than 3870", err_address
#print "WARNING: number of address greater than FFFF", overflow_address
print "data loaded"



figure()
plot(xCdOn,yCdOn,'.r',xCdOff,yCdOff,'.b')
hold(True)
plot(xEM1L,yEM1L,'.g',xEM1H,yEM1H,'.g')
plot(xEM2L,yEM2L,'.y',xEM2H,yEM2H,'.y')
plot(xEM3L,yEM3L,'.g',xEM3H,yEM3H,'.g')
plot(xEM4L,yEM4L,'.y',xEM4H,yEM4H,'.y')
plot(xIFOn,yIFOn,'.m',xIFOff,yIFOff,'.c')
show()

#p.plot(xCdOn,yCdOn,'.r',xCdOff,yCdOff,'.b')
#p.hold(True)
#p.plot(xEM1L,yEM1L,'.g',xEM1H,yEM1H,'.g')
#p.plot(xEM2L,yEM2L,'.y',xEM2H,yEM2H,'.y')
#p.plot(xEM3L,yEM3L,'.g',xEM3H,yEM3H,'.g')
#p.plot(xEM4L,yEM4L,'.y',xEM4H,yEM4H,'.y')
#p.plot(xIFOn,yIFOn,'.m',xIFOff,yIFOff,'.c')
#p.show()
#ax = host_subplot(111, axes_class=AA.Axes)#
#fig = plt.figure()
#ax = fig.add_subplot(111)

# plot of the events in Raster form
# extracting metapixels in required range

limitPixelMin = 12 * 3
limitPixelMax = 12 * 2
#timestampMin  = limitPixelMin
#timestampMax  = limitPixelMax 
#p.plot(xRaster[0:12],yRaster[0:12],'o',xRaster[12:24],yRaster[12:24],'ro',xRaster[24:50],yRaster[24:50],'mo')
#p.plot(xRaster,yRaster,'o')

#for i, ax in enumerate(event.canvas.figure.axes): # Get the axes
#	ax.set_xlim(lims)
#ax.set_ylim()
#p.axis([timestampMin, timestampMax,metaPixelMin * 12 , (metaPixelMax + 1) * 12])
#pylab.show()
#plt.draw()
#p.show()

# plot of the event in the Raster form 
#rasterPlot.plot(x[:], y[0:12],'o',x[], y[12:24],'ro',x, y[24:36],'mo')
#rasterPlot.show()

