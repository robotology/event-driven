#!/usr/bin/python 

import sys
import os
import array
import getopt,string 
from numpy import *
from decimal import *
from pylab import *
#from string import *
from string import Template
import time

# ------------------------------------------------------------ #
# Library of routines for analysis and plotting of asv data
# -- filename contains the events' stream to be analysed
# -- typePlot can be 'noPlot', 'Cd', 'CdEm' for raster plots and '2D' to visualize the active addresses in the chip, 'ISI' to visualize the results of ISI analysis
# -- typeAnalysis can be 'ISI' to measure mean and std of time intervals between Cd-EMH and EMH-EML and to report some statistics on the sequences of spike times (e.g. if in the train a EML spike has been found without a preceding EMH)

# ------------------------------------------------------------ #
# default settings for printable plots
font = {'family':'sans-serif','weight':'normal','size':18}
rc('font',**font)
rc('lines',linewidth = 2)
# ------------------------------------------------------------ #
def readfile(filename,buffer):
	'''Print a file to the standard output.'''
	arr = []
	f = file(filename)
	while True:
		line = f.readline()
		if len(line) == 0:
			break
		arr.append(line)
		buffer = buffer + 1
	f.close()
	return arr
# ------------------------------------------------------------ #
def dec2hex(n):
	#
	"""return the hexadecimal string representation of integer n"""
	#
	return "%X" % n
 
# ------------------------------------------------------------ #
def hex2dec(s):
	#
	"""return the integer value of a hexadecimal string s"""
	#
	return int(s, 16)

# ------------------------------------------------------------ #
def int2bin(n, count=24):
	#
	"""returns the binary of integer n, using count number of digits"""
	#
	return "".join([str((n >> y) & 1) for y in range(count-1, -1, -1)])
# ------------------------------------------------------------ #

def int2bin_array(n, count=24):
	#
	"""returns the binary of integer n, using count number of digits"""
	#
	a = zeros(24,int)
	for y in range(count-1, -1, -1):
		a[-1-y] = (n >> y) & 1
	return a
# ------------------------------------------------------------ #
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
# ------------------------------------------------------------ #
def asvData(filename,typePlot,typeAnalysis):
	# typePlot ['Cd','CdEm','CdEmLine','2D','ISI'] or ['None']
	# typeAnalysis 'ISI' or 'None'
	print "load data from ", filename
	os.system("rm -f /home/icub/asv/dump/asvTOT.txt")
	sCmdTemp = Template ("cat /home/icub/asv/dump/asvLUT.txt $fName >> /home/icub/asv/dump/asvTOT.txt")
	sCmd = sCmdTemp.substitute(fName = filename)
	os.system(sCmd)

	xMask   = hex2dec("FF")
	yMask   = hex2dec("7F00")
	xShift  = 0
	yShift  = 8
	xoffset = -112
	yoffset = -56
	reverseBits = 1	
        #initilization
	metaPixelMax = 3
	metaPixelMin = 0
	timestampMax = 2000000
	timestampMin = 0
	sizeLUT = 10368
	countEvent = 0

	print "reading from file"
	#execfile("./readFile.py")
	data = readfile("/home/icub/asv/dump/asvTOT.txt", 0)

	size = len(data)
	lenData = size - sizeLUT 
	print "data length", lenData
	
	x = zeros(size, int)
	y = zeros(size, int)
	xRaster = zeros(lenData, int)
	yRaster = zeros(lenData, int)
	
	if typePlot != 'noPlot':
		xCdOff = zeros(lenData, int); yCdOff = zeros(lenData, int)
		xCdOn = zeros(lenData, int); yCdOn = zeros(lenData, int)
		xEM1L = zeros(lenData, int); yEM1L = zeros(lenData, int)
		xEM1H = zeros(lenData, int); yEM1H = zeros(lenData, int)
		xEM2L = zeros(lenData, int); yEM2L = zeros(lenData, int)
		xEM2H = zeros(lenData, int); yEM2H = zeros(lenData, int)
		xEM3L = zeros(lenData, int); yEM3L = zeros(lenData, int)
		xEM3H = zeros(lenData, int); yEM3H = zeros(lenData, int)
		xEM4L = zeros(lenData, int); yEM4L = zeros(lenData, int)
		xEM4H = zeros(lenData, int); yEM4H = zeros(lenData, int)
		xIFOn = zeros(lenData, int); yIFOn = zeros(lenData, int)
		xIFOff = zeros(lenData, int); yIFOff = zeros(lenData, int)
	if typeAnalysis == 'ISI':
		timeEvents = zeros((24,24,lenData))
		typeEvents = zeros((24,24,lenData))
		indexEvent = 0

	metax = zeros(sizeLUT, int)
	metay = zeros(sizeLUT, int)
	type_pxl  = zeros(sizeLUT, int)
	polar = zeros(sizeLUT, int)
	firstTimestamp = 0

	timestamp = 0
	address = 0
	# execution
	for i in range (0,size):
		data2 = data[i].rsplit("\n")
		dataS = data2[0].split(' ')
		if dataS[0]=="#" :
			xchip = Decimal(dataS[1]);
			ychip = Decimal(dataS[2]);
			position = xchip * 72 + ychip
			metax[position] = Decimal(dataS[3])
			metay[position] = Decimal(dataS[4])
			polar[position] = Decimal(dataS[5])
			type_pxl [position] = Decimal(dataS[6])
		else:	
			address = hex2dec(dataS[0])
			timestamp = hex2dec(dataS[1])
			
			if countEvent == 0 :
				firstTimestamp = timestamp
				lastTimestamp  = timestamp
			x[countEvent] = (address & xMask) 
			x[countEvent] = x[countEvent] >> xShift
			y[countEvent] = (address & yMask)
			y[countEvent] = y[countEvent] >> yShift
			yflip = flipBits(y[countEvent],7)

			y[countEvent] = yflip + yoffset
			x[countEvent] = x[countEvent]  + xoffset
		
			position =  x[countEvent] * 72 + y[countEvent]
	
			mx = metax[position] 
			my = metay[position] 
			po = polar[position] 
			ty = type_pxl[position] 		
			# LUT: type_pxl
			# 0 CD, 1 EM1, 2 EM2, 3 EM3, 4 EM4, 5 IF
			# LTU: polarity
			# CD and IF 0 -> Off, 1 -> On, EM 0 -> Low, 1 -> High
			yRaster[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
			xRaster[countEvent] = (timestamp - firstTimestamp) * 160 #ns
			if typePlot != 'noPlot':
				# divide the events in different types, but keep the same y coordinate
				if ty * 2 + po  == 0:
					yCdOff[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xCdOff[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 1:
					yCdOn[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xCdOn[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 2:
					yEM1L[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM1L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 3:
					yEM1H[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM1H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 4:
					yEM2L[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM2L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 5:
					yEM2H[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM2H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty  * 2 + po== 6:
					yEM3L[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM3L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty * 2 + po == 7:
					yEM3H[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM3H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty  * 2 + po == 8:
					yEM4L[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM4L[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty  * 2 + po == 9:
					yEM4H[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xEM4H[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty  * 2 + po == 10:
					yIFOff[countEvent] = (mx * 24 + my) * 12 + ty * 2 + po		
					xIFOff[countEvent] = (timestamp - firstTimestamp) * 160 #ns
				elif ty  * 2 + po == 11:
					yIFOn[countEvent]  = (mx * 24 + my) * 12 + ty * 2 + po		
					xIFOn[countEvent]  = (timestamp - firstTimestamp) * 160 #ns
			if typeAnalysis == 'ISI': 
				# 3D arrays with time-stamps and type of event divided into meta-pixels (first two indexes)
				# first zero element of timeEvents (to put events in a consecutive manner avoiding zeros 
				# in between consecutive events)
				indexEvent = nonzero(timeEvents[mx,my] == 0)
				timeEvents[mx,my,indexEvent[0][0]] = xRaster[countEvent]
				typeEvents[mx,my,indexEvent[0][0]] = ty * 2 + po

			countEvent = countEvent + 1


	print "data loaded"
	# print 'typeAnalysis', typeAnalysis, 'if', typeAnalysis == 'ISI'
	if typeAnalysis == 'ISI':
		print "ISI analysis"
		# initialize flags
		flagEM1H = 0; flagEM2H = 0; flagEM3H = 0; flagEM4H = 0
		flagEM1H_L = 0;	flagEM2H_L = 0; flagEM3H_L = 0; flagEM4H_L = 0
		# initialize counters
		countCdOff_EML	= zeros((24,24)); countCdOff_repEMH = zeros((24,24))
		countCdOff_noEM = zeros((24,24)); countCdOff_EMH = zeros((24,24))
		countCdOn_EML	= zeros((24,24)); countCdOn_repEMH = zeros((24,24))
		countCdOn_noEM = zeros((24,24)); countCdOn_EMH = zeros((24,24))
		countEMH_EML = zeros((24,24)); countCdOn = zeros((24,24))
		countCdOff = zeros((24,24)); countEvents = zeros((24,24))

		timeCdOff_EM1H = zeros((24,24,lenData))
		timeCdOffEM1H_EM1L = zeros((24,24,lenData))
		timeCdOff_EM2H = zeros((24,24,lenData))
		timeCdOffEM2H_EM2L = zeros((24,24,lenData))
		timeCdOff_EM3H = zeros((24,24,lenData))
		timeCdOffEM3H_EM3L = zeros((24,24,lenData))
		timeCdOff_EM4H = zeros((24,24,lenData))
		timeCdOffEM4H_EM4L = zeros((24,24,lenData))
		timeCdOn_EM1H = zeros((24,24,lenData))
		timeCdOnEM1H_EM1L = zeros((24,24,lenData))
		timeCdOn_EM2H = zeros((24,24,lenData))
		timeCdOnEM2H_EM2L = zeros((24,24,lenData))
		timeCdOn_EM3H = zeros((24,24,lenData))
		timeCdOnEM3H_EM3L = zeros((24,24,lenData))
		timeCdOn_EM4H = zeros((24,24,lenData))
		timeCdOnEM4H_EM4L = zeros((24,24,lenData))

		for ix in range (0,24): # meta-pixel x coordinate
			for iy in range (0,24): # meta-pixel y coordinate
				# print 'ix,iy',ix, iy
				maxEvents = nonzero(timeEvents[ix,iy] == 0)
				for indEvent in range (0,maxEvents[0][0]):
					countEvents[ix,iy] = countEvents[ix,iy] + 1
					# --------- is it a CdOff spike? ----------- #
					if typeEvents[ix,iy,indEvent] == 0: # CdOff event
						countCdOff[ix,iy] = countCdOff[ix,iy] + 1
						# print 'found cdOn, indEvent = ',indEvent
						# print 'next typeEvents = ',typeEvents[ix,iy,indEvent:10]
						# search for EmHigh events after the CdOff spike until a new Cd event is found (either On or Off)
						i = 1 # start from index after CdOff spike
						flagEM1H = 0; flagEM2H = 0; flagEM3H = 0; flagEM4H = 0 # reset flags
						flagEM1L = 0; flagEM2L = 0; flagEM3L = 0; flagEM4L = 0 # reset flags
						while (typeEvents[ix,iy,indEvent + i] != 0 and typeEvents[ix,iy,indEvent + i] != 1) and timeEvents[ix,iy,indEvent + i] != 0 and indEvent + i < lenData - 1:
					
							tE = copy(typeEvents[ix,iy,indEvent + i])	
							# print 'search for EMH, i = ',i, 'type event =',tE
							# --- is the next spike the first EM1H? --- #
							if tE == 3 and flagEM1H == 0: # EM1H
								timeCdOff_EM1H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM1H = 1
								countCdOff_EMH[ix,iy] = countCdOff_EMH[ix,iy] + 1
								# print 'first EM1H, i = ',i
								# search for Em1Low events after the Em1High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM1H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML, k1 =',k1
									if typeEvents[ix,iy,indEvent + k1] == 2: # EM1L
										timeCdOffEM1H_EM1L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM1H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM1H-EM1L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM1L
						
							# --- is the next spike the first EM2H? --- #
							elif tE == 5 and flagEM2H == 0: # EM2H	
								timeCdOff_EM2H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM2H = 1
								countCdOff_EMH[ix,iy] = countCdOff_EMH[ix,iy] + 1
								# print 'first EM2H'
								# search for Em2Low events after the Em2High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM2H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 4: # EM2L
										timeCdOffEM2H_EM2L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM2H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM2H-EM2L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM2L
						
							# --- is the next spike the first EM3H? --- #
							elif tE == 7 and flagEM3H == 0: # EM3H	
								timeCdOff_EM3H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM3H = 1
								countCdOff_EMH[ix,iy] = countCdOff_EMH[ix,iy] + 1
								# search for Em3Low events after the EmHigh spike until a new Cd event is found
								k1 = i + 1 # start from index after EM3H spike
								# print 'first EM3H'
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 6: # EM3L
										timeCdOffEM3H_EM3L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM3H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM3H-EM3L found'
										break 						
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM3L
				
							# --- is the next spike the first EM4H? --- #
							elif tE == 9 and flagEM4H == 0: # EM4H	
								timeCdOff_EM4H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM4H = 1
								countCdOff_EMH[ix,iy] = countCdOff_EMH[ix,iy] + 1
								# print 'first EM4H'
								# search for Em4Low events after the Em4High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM4H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 8: # EM4L
										timeCdOffEM4H_EM4L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM4H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM4H-EM4L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM4L
					
							# --- is the next spike EMnL? --- #
							elif tE == 2:
								if flagEM1H_L == 1: 
									flagEM1H_L = 0
								else:
									countCdOff_EML[ix,iy] = countCdOff_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 4:
								if flagEM2H_L == 1:
									flagEM2H_L = 0	
								else:
									countCdOff_EML[ix,iy] = countCdOff_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 6:
								if flagEM3H_L == 1:
									flagEM3H_L = 0	
								else:
									countCdOff_EML[ix,iy] = countCdOff_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 8:
								if flagEM4H_L == 1:
									flagEM4H_L = 0
								else:
									countCdOff_EML[ix,iy] = countCdOff_EML[ix,iy] + 1
									# print 'single EML!'
							# --- is the next spike a repetition of EMnH? --- #
							elif (tE == 3 and flagEM1H == 1) or (tE == 5 and flagEM2H == 1) or (tE == 7 and flagEM3H == 1) or(tE == 9 and flagEM4H == 1):
								countCdOff_repEMH[ix,iy] = countCdOff_repEMH[ix,iy] + 1
								# print 'repeated EMH'
							# --- no spikes after CdOff? --- #
							else: 
								countCdOff_noEM[ix,iy] = countCdOff_noEM[ix,iy] + 1
								# print 'no spikes!'
							i = i + 1 # increment i to scan the spikes until the while is true (until a CdOff or CdOn is found)
				
					# --------- is it a CdOn spike? ----------- #
					elif typeEvents[ix,iy,indEvent] == 1: # CdOn event
						countCdOn[ix,iy] = countCdOn[ix,iy] + 1
						# print 'found cdOn, indEvent = ',indEvent
						# print 'next typeEvents = ',typeEvents[ix,iy,indEvent:10]
						# search for EmHigh events after the CdOn spike until a new Cd event is found (either On or Off)
						i = 1 # start from index after CdOn spike
						flagEM1H = 0; flagEM2H = 0; flagEM3H = 0; flagEM4H = 0 # reset flags
						flagEM1L = 0; flagEM2L = 0; flagEM3L = 0; flagEM4L = 0 # reset flags
						while (typeEvents[ix,iy,indEvent + i] != 0 and typeEvents[ix,iy,indEvent + i] != 1) and timeEvents[ix,iy,indEvent + i] != 0 and indEvent + i < lenData - 1:
					
							tE = copy(typeEvents[ix,iy,indEvent + i])	
							# print 'search for EMH, i = ',i, 'type event =',tE
							# --- is the next spike the first EM1H? --- #
							if tE == 3 and flagEM1H == 0: # EM1H
								timeCdOn_EM1H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM1H = 1
								countCdOn_EMH[ix,iy] = countCdOn_EMH[ix,iy] + 1
								# print 'first EM1H, i = ',i
								# search for Em1Low events after the Em1High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM1H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML, k1 =',k1
									if typeEvents[ix,iy,indEvent + k1] == 2: # EM1L
										timeCdOnEM1H_EM1L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM1H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM1H-EM1L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM1L
						
							# --- is the next spike the first EM2H? --- #
							elif tE == 5 and flagEM2H == 0: # EM2H	
								timeCdOn_EM2H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM2H = 1
								countCdOn_EMH[ix,iy] = countCdOn_EMH[ix,iy] + 1
								# print 'first EM2H'
								# search for Em2Low events after the Em2High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM2H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 4: # EM2L
										timeCdOnEM2H_EM2L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM2H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM2H-EM2L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM2L
						
							# --- is the next spike the first EM3H? --- #
							elif tE == 7 and flagEM3H == 0: # EM3H	
								timeCdOn_EM3H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM3H = 1
								countCdOn_EMH[ix,iy] = countCdOn_EMH[ix,iy] + 1
								# search for Em3Low events after the EmHigh spike until a new Cd event is found
								k1 = i + 1 # start from index after EM3H spike
								# print 'first EM3H'
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 6: # EM3L
										timeCdOnEM3H_EM3L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM3H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM3H-EM3L found'
										break 						
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM3L
				
							# --- is the next spike the first EM4H? --- #
							elif tE == 9 and flagEM4H == 0: # EM4H	
								timeCdOn_EM4H[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + i] - timeEvents[ix,iy,indEvent]
								flagEM4H = 1
								countCdOn_EMH[ix,iy] = countCdOn_EMH[ix,iy] + 1
								# print 'first EM4H'
								# search for Em4Low events after the Em4High spike until a new Cd event is found
								k1 = i + 1 # start from index after EM4H spike
								while (typeEvents[ix,iy,indEvent + k1] != 0 and typeEvents[ix,iy,indEvent + k1] != 1) and indEvent + k1 < lenData - 1:
									# print 'search for EML'
									if typeEvents[ix,iy,indEvent + k1] == 8: # EM4L
										timeCdOnEM4H_EM4L[ix,iy,indEvent] = timeEvents[ix,iy,indEvent + k1] - timeEvents[ix,iy,indEvent + i]
										flagEM4H_L = 1
										countEMH_EML[ix,iy] = countEMH_EML[ix,iy] + 1
										# print 'EM4H-EM4L found'
										break 
									else: k1 = k1 + 1 # we keep scanning the array with k1 looking for EM4L
					
							# --- is the next spike EMnL? --- #
							elif tE == 2:
								if flagEM1H_L == 1: 
									flagEM1H_L = 0
								else:
									countCdOn_EML[ix,iy] = countCdOn_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 4:
								if flagEM2H_L == 1:
									flagEM2H_L = 0	
								else:
									countCdOn_EML[ix,iy] = countCdOn_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 6:
								if flagEM3H_L == 1:
									flagEM3H_L = 0	
								else:
									countCdOn_EML[ix,iy] = countCdOn_EML[ix,iy] + 1
									# print 'single EML!'
							elif tE == 8:
								if flagEM4H_L == 1:
									flagEM4H_L = 0
								else:
									countCdOn_EML[ix,iy] = countCdOn_EML[ix,iy] + 1
									# print 'single EML!'
							# --- is the next spike a repetition of EMnH? --- #
							elif (tE == 3 and flagEM1H == 1) or (tE == 5 and flagEM2H == 1) or (tE == 7 and flagEM3H == 1) or(tE == 9 and flagEM4H == 1):
								countCdOn_repEMH[ix,iy] = countCdOn_repEMH[ix,iy] + 1
								# print 'repeated EMH'
							# --- no spikes after CdOn? --- #
							else: 
								countCdOn_noEM[ix,iy] = countCdOn_noEM[ix,iy] + 1
								# print 'no spikes!'
							i = i + 1 # increment i to scan the spikes until the while is true (until a CdOff or CdOn is found)
				
		totCdOn = sum(countCdOn); totCdOn_EMH = sum(countCdOn_EMH)
		totCdOn_EML = sum(countCdOn_EML); totCdOn_repEMH = sum(countCdOn_repEMH)
		totCdOn_noEM = sum(countCdOn_noEM)

		totCdOff = sum(countCdOff); totCdOff_EMH = sum(countCdOff_EMH)
		totCdOff_EML = sum(countCdOff_EML); totCdOff_noEM = sum(countCdOff_noEM)
		totCdOff_repEMH = sum(countCdOff_repEMH)

		totEMH_EML = sum(countEMH_EML)

		# print 'totCdOn = ', totCdOn
		# print 'totCdOn_EMH = ', totCdOn_EMH
		# print 'totCdOn_EML = ', totCdOn_EML
		# print 'totCdOn_repEMH = ', totCdOn_repEMH
		# print 'totCdOn_noEM = ', totCdOn_noEM

		# print 'totCdOff = ', totCdOff
		# print 'totCdOff_EMH = ', totCdOff_EMH
		# print 'totCdOff_EML = ', totCdOff_EML
		# print 'totCdOff_noEM = ', totCdOff_noEM
		# print 'totCdOff_repEMH = ', totCdOff_repEMH

		# print 'totEMH_EML = ', totEMH_EML
		
		a = []; b = []; c = []; d = []; e = []; f = []; g = []; h = []
		for ix in range (0,24):
			for iy in range (0,24):
				if len(nonzero(timeCdOnEM1H_EM1L[ix,iy])[0])!= 0 :
					trim_a = timeCdOnEM1H_EM1L[ix,iy,nonzero(timeCdOnEM1H_EM1L[ix,iy])]
					a = concatenate((a,trim_a[0]))
				if len(nonzero(timeCdOnEM2H_EM2L[ix,iy])[0])!= 0 :
					trim_b = timeCdOnEM2H_EM2L[ix,iy,nonzero(timeCdOnEM2H_EM2L[ix,iy])]
					b = concatenate((b,trim_b[0]))
				if len(nonzero(timeCdOnEM3H_EM3L[ix,iy])[0])!= 0 :
					trim_c = timeCdOnEM3H_EM3L[ix,iy,nonzero(timeCdOnEM1H_EM1L[ix,iy])]
					c = concatenate((c,trim_c[0]))
				if len(nonzero(timeCdOnEM4H_EM4L[ix,iy])[0])!= 0 :
					trim_d = timeCdOnEM4H_EM4L[ix,iy,nonzero(timeCdOnEM4H_EM4L[ix,iy])]
					d = concatenate((d,trim_d[0]))
				if len(nonzero(timeCdOn_EM1H[ix,iy])[0])!= 0 :
					trim_e = timeCdOn_EM1H[ix,iy,nonzero(timeCdOn_EM1H[ix,iy])]
					e = concatenate((e,trim_e[0]))
				if len(nonzero(timeCdOn_EM2H[ix,iy])[0])!= 0 :
					trim_f = timeCdOn_EM2H[ix,iy,nonzero(timeCdOn_EM2H[ix,iy])]
					f = concatenate((f,trim_f[0]))
				if len(nonzero(timeCdOn_EM3H[ix,iy])[0])!= 0 :
					trim_g = timeCdOn_EM3H[ix,iy,nonzero(timeCdOn_EM3H[ix,iy])]
					g = concatenate((g,trim_g[0]))
				if len(nonzero(timeCdOn_EM4H[ix,iy])[0])!= 0 :
					trim_h = timeCdOn_EM4H[ix,iy,nonzero(timeCdOn_EM4H[ix,iy])]
					h = concatenate((h,trim_h[0]))

		meanTimeCdOnEM1H_EM1L = mean(a)*1e-6
		stdTimeCdOnEM1H_EM1L = std(a)*1e-6
		# print 'TimeCdOnEM1H_EM1L: mean = ', mean(a)*1e-6, '(ms), std = ', std(a)*1e-6, '(ms)'
		meanTimeCdOnEM2H_EM2L = mean(b)*1e-6
		stdTimeCdOnEM2H_EM2L = std(b)*1e-6
		# print 'TimeCdOnEM2H_EM2L: mean = ', mean(b)*1e-6, '(ms), std = ', std(b)*1e-6, '(ms)'
		meanTimeCdOnEM3H_EM3L = mean(c)*1e-6
		stdTimeCdOnEM3H_EM3L = std(c)*1e-6
		# print 'TimeCdOnEM3H_EM3L: mean = ', mean(c)*1e-6, '(ms), std = ', std(c)*1e-6, '(ms)'
		meanTimeCdOnEM4H_EM4L = mean(d)*1e-6
		stdTimeCdOnEM4H_EM4L = std(d)*1e-6
		# print 'TimeCdOnEM4H_EM4L: mean = ', mean(d)*1e-6, '(ms), std = ', std(d)*1e-6, '(ms)'
		meanTimeCdOn_EM1H = mean(e)*1e-6
		stdTimeCdOn_EM1H = std(e)*1e-6
		# print 'TimeCdOn_EM1H: mean = ', mean(e)*1e-6, '(ms), std = ', std(e)*1e-6, '(ms)'
		meanTimeCdOn_EM2H = mean(f)*1e-6
		stdTimeCdOn_EM2H = std(f)*1e-6
		# print 'TimeCdOn_EM2H: mean = ', mean(f)*1e-6, '(ms), std = ', std(f)*1e-6, '(ms)'
		meanTimeCdOn_EM3H = mean(g)*1e-6
		stdTimeCdOn_EM3H = std(g)*1e-6
		# print 'TimeCdOn_EM3H: mean = ', mean(g)*1e-6, '(ms), std = ', std(g)*1e-6, '(ms)'
		meanTimeCdOn_EM4H = mean(h)*1e-6
		stdTimeCdOn_EM4H = std(h)*1e-6
		# print 'TimeCdOn_EM4H: mean = ', mean(h)*1e-6, '(ms), std = ', std(h)*1e-6, '(ms)'

		a = []; b = []; c = []; d = []; e = []; f = []; g = []; h = []
		for ix in range (0,24):
			for iy in range (0,24):
				if len(nonzero(timeCdOffEM1H_EM1L[ix,iy])[0])!= 0 :
					trim_a = timeCdOffEM1H_EM1L[ix,iy,nonzero(timeCdOffEM1H_EM1L[ix,iy])]
					a = concatenate((a,trim_a[0]))
				if len(nonzero(timeCdOffEM2H_EM2L[ix,iy])[0])!= 0 :
					trim_b = timeCdOffEM2H_EM2L[ix,iy,nonzero(timeCdOffEM2H_EM2L[ix,iy])]
					b = concatenate((b,trim_b[0]))
				if len(nonzero(timeCdOffEM3H_EM3L[ix,iy])[0])!= 0 :
					trim_c = timeCdOffEM3H_EM3L[ix,iy,nonzero(timeCdOffEM1H_EM1L[ix,iy])]
					c = concatenate((c,trim_c[0]))
				if len(nonzero(timeCdOffEM4H_EM4L[ix,iy])[0])!= 0 :
					trim_d = timeCdOffEM4H_EM4L[ix,iy,nonzero(timeCdOffEM4H_EM4L[ix,iy])]
					d = concatenate((d,trim_d[0]))
				if len(nonzero(timeCdOff_EM1H[ix,iy])[0])!= 0 :
					trim_e = timeCdOff_EM1H[ix,iy,nonzero(timeCdOff_EM1H[ix,iy])]
					e = concatenate((e,trim_e[0]))
				if len(nonzero(timeCdOff_EM2H[ix,iy])[0])!= 0 :
					trim_f = timeCdOff_EM2H[ix,iy,nonzero(timeCdOff_EM2H[ix,iy])]
					f = concatenate((f,trim_f[0]))
				if len(nonzero(timeCdOff_EM3H[ix,iy])[0])!= 0 :
					trim_g = timeCdOff_EM3H[ix,iy,nonzero(timeCdOff_EM3H[ix,iy])]
					g = concatenate((g,trim_g[0]))
				if len(nonzero(timeCdOff_EM4H[ix,iy])[0])!= 0 :
					trim_h = timeCdOff_EM4H[ix,iy,nonzero(timeCdOff_EM4H[ix,iy])]
					h = concatenate((h,trim_h[0]))

		meanTimeCdOffEM1H_EM1L = mean(a)*1e-6
		stdTimeCdOffEM1H_EM1L = std(a)*1e-6
		# print 'TimeCdOffEM1H_EM1L: mean = ', mean(a)*1e-6, '(ms), std = ', std(a)*1e-6, '(ms)'
		meanTimeCdOffEM2H_EM2L = mean(b)*1e-6
		stdTimeCdOffEM2H_EM2L = std(b)*1e-6
		# print 'TimeCdOffEM2H_EM2L: mean = ', mean(b)*1e-6, '(ms), std = ', std(b)*1e-6, '(ms)'
		meanTimeCdOffEM3H_EM3L = mean(c)*1e-6
		stdTimeCdOffEM3H_EM3L = std(c)*1e-6
		# print 'TimeCdOffEM3H_EM3L: mean = ', mean(c)*1e-6, '(ms), std = ', std(c)*1e-6, '(ms)'
		meanTimeCdOffEM4H_EM4L = mean(d)*1e-6
		stdTimeCdOffEM4H_EM4L = std(d)*1e-6
		# print 'TimeCdOffEM4H_EM4L: mean = ', mean(d)*1e-6, '(ms), std = ', std(d)*1e-6, '(ms)'
		meanTimeCdOff_EM1H = mean(e)*1e-6
		stdTimeCdOff_EM1H = std(e)*1e-6
		# print 'TimeCdOff_EM1H: mean = ', mean(e)*1e-6, '(ms), std = ', std(e)*1e-6, '(ms)'
		meanTimeCdOff_EM2H = mean(f)*1e-6
		stdTimeCdOff_EM2H = std(f)*1e-6
		# print 'TimeCdOff_EM2H: mean = ', mean(f)*1e-6, '(ms), std = ', std(f)*1e-6, '(ms)'
		meanTimeCdOff_EM3H = mean(g)*1e-6
		stdTimeCdOff_EM3H = std(g)*1e-6
		# print 'TimeCdOff_EM3H: mean = ', mean(g)*1e-6, '(ms), std = ', std(g)*1e-6, '(ms)'
		meanTimeCdOff_EM4H = mean(h)*1e-6
		stdTimeCdOff_EM4H = std(h)*1e-6
		# print 'TimeCdOff_EM4H: mean = ', mean(h)*1e-6, '(ms), std = ', std(h)*1e-6, '(ms)'

	if typePlot != 'noPlot':
		hor_grid = zeros(24*24)
		i = 0
		for ax in range (0,24):
			for ay in range (0,24):
				hor_grid[i] = (ax * 24 + ay) * 12     
				i=i+1
		
		for j in range (0, len(typePlot)):
		
			if typePlot[j] == 'Cd' or typePlot[j] == 'All':
				figure()
				plot(xCdOn,yCdOn,'.r',xCdOff,yCdOff,'.b')
				title('CD',fontsize = 'x-large')
				hold('on')
				xlabel('Time (ns)',fontsize = 'x-large')
				ylabel('Address',fontsize = 'x-large')
				# for i in range (0,len(hor_grid)):
				# 	axhline(y=hor_grid[i])

			elif typePlot[j] == 'CdEm' or typePlot[j] == 'All' or typePlot[j] == 'CdEmLine':
				figure()
				plot(xCdOn,yCdOn,color = 'grey', marker = '.', linestyle = '')
				hold('on')
				plot(xCdOff,yCdOff,color = 'black', marker = '.', linestyle = '')
				
				plot(xEM1H,yEM1H,color = 'm', marker = '.', linestyle = '')
				plot(xEM1L,yEM1L,color = 'm', marker = '|', linestyle = '')
				plot(xEM2H,yEM2H,color = 'c', marker = '.', linestyle = '')
				plot(xEM2L,yEM2L,color = 'c', marker = '|', linestyle = '')
				plot(xEM3H,yEM3H,color = 'r', marker = '.', linestyle = '')
				plot(xEM3L,yEM3L,color = 'r', marker = '|', linestyle = '')
				plot(xEM4H,yEM4H,color = 'b', marker = '.', linestyle = '')
				plot(xEM4L,yEM4L,color = 'b', marker = '|', linestyle = '')
				xlabel('Time (ns)',fontsize = 'x-large')
				ylabel('Address',fontsize = 'x-large')
				title('CdEm',fontsize = 'x-large')
				if typePlot[j] == 'CdEmLine':
					for i in range (0,len(hor_grid)):
						axhline(y=hor_grid[i])
			elif typePlot[j] == '2D' or typePlot[j] == 'All':
				figure()
				plot(x,y,'o')
				hold('on')
				x_minor_grid = arange(0,145,2)-0.5
				x_major_grid = arange(0,145,12)-0.5
				for i in range (0,len(x_minor_grid)):
					axvline(x = x_minor_grid[i],linewidth = 1, color = 'r')
				for i in range (0,len(x_major_grid)):
					axvline(x = x_major_grid[i],linewidth = 2, color = 'r')
				

				y_minor_grid = arange(0,73,1)-0.5
				y_major_grid = arange(0,73,6)-0.5
				for i in range (0,len(y_minor_grid)):
					axhline(y = y_minor_grid[i],linewidth = 1, color = 'r')
				for i in range (0,len(y_major_grid)):
					axhline(y = y_major_grid[i],linewidth = 2, color = 'r')

				xlim(-1,144)
				ylim(-1,72)
			elif typePlot[j] == 'ISI' or typePlot[j] == 'All':
				figure()
				hold('on')
				errorbar(range(1,5),[meanTimeCdOn_EM1H,meanTimeCdOn_EM2H,meanTimeCdOn_EM3H,meanTimeCdOn_EM4H],[stdTimeCdOn_EM1H,stdTimeCdOn_EM2H,stdTimeCdOn_EM3H,stdTimeCdOn_EM4H],label = 'CdOn - EmH',linewidth = 2)
				errorbar(range(1,5),[meanTimeCdOnEM1H_EM1L,meanTimeCdOnEM2H_EM2L,meanTimeCdOnEM3H_EM3L,meanTimeCdOnEM4H_EM4L],[stdTimeCdOnEM1H_EM1L,stdTimeCdOnEM2H_EM2L,stdTimeCdOnEM3H_EM3L,stdTimeCdOnEM4H_EM4L], label = 'EmH - EmL (CdOn)',linewidth = 2)
				errorbar(range(1,5),[meanTimeCdOff_EM1H,meanTimeCdOff_EM2H,meanTimeCdOff_EM3H,meanTimeCdOff_EM4H],[stdTimeCdOff_EM1H,stdTimeCdOff_EM2H,stdTimeCdOff_EM3H,stdTimeCdOff_EM4H], label = 'CdOff - EmH',linewidth = 2)
				errorbar(range(1,5),[meanTimeCdOffEM1H_EM1L,meanTimeCdOffEM2H_EM2L,meanTimeCdOffEM3H_EM3L,meanTimeCdOffEM4H_EM4L],[stdTimeCdOffEM1H_EM1L,stdTimeCdOffEM2H_EM2L,stdTimeCdOffEM3H_EM3L,stdTimeCdOffEM4H_EM4L], label = 'EmH - EmL (CdOff)',linewidth = 2)
				xlim(0,5)
				xlabel('EMn',fontsize = 'x-large')
				ylabel('$\Delta$t (ms)',fontsize = 'x-large')
				suptitle(totEMH_EML/(totCdOn+totCdOff))
				legend()
				
		show()
# ------------------ BiasGen -------------- #
def setBias(biasName,biasCurrent):
	# transforms a human readable current into the numeric value needed for programming the biases
	[binValue,biasValue] = current2int(biasCurrent)
	sCmdTemp = Template("pythonInterface --rpcport /asvGrabber --request 'command:set,command:$bName,int:$bValue;'")
	sCmd = sCmdTemp.substitute(bName = biasName,bValue = biasValue)
	os.system(sCmd)
	
def progBiases():
	# program biases
	os.system("pythonInterface --rpcport /asvGrabber --request 'command:prog,command:left;'")
	print 'bias programmed'
	time.sleep(2)

def getBias(biasName):
	# transforms into human readable current the value read from the programmed bias
	sCmdTemp = Template("pythonInterface --rpcport /asvGrabber --request 'command:get,command:$bName;'")
	sCmd = sCmdTemp.substitute(bName = biasName)
	biasValue = os.system(sCmd)
	# biasCurrent = int2current(biasValue)
	# return biasCurrent

def int2current(biasInt):
	# biasInt read from "get bias" min 0, max 16777215
	# initialization
	masterCurrent = 1.3e-5 # 13uA
	biasSplit = exp2(arange(-1,-25,-1))
	binBias = int2bin_array(biasInt)
	biasCurrent = masterCurrent * sum(biasSplit*binBias)
	return biasCurrent

def current2int(biasCurrent):
	# inverse find biasInt to send to asvGrabber, from needed current
	binBias = zeros(24,int)
	biasSplit = exp2(arange(-1,-25,-1))
	masterCurrent = 1.3e-5 # 13uA
	for i in range(1,25):
		if (biasCurrent/masterCurrent) - sum(biasSplit*binBias) >= exp2(-i):
			binBias[i-1] = 1
		# print 'Ic/Im-sum(biasSplit*binBias)', biasCurrent/masterCurrent-sum(biasSplit*binBias)
		# print '$2^{-i}$',exp2(-i)
		# print 'binBias', binBias
	
	intBias = sum(binBias * exp2(arange(23,-1,-1)))
	return binBias,intBias
# --------------------------------------------- #
def asvSetup(defBiases):
	# load biases from file fileName USE ABSOLUTE PATH!!!!!
	# defBiases = "/home/icub/asv/biases/DefaultCD_EM.txt" # file of default values for cd and em pixels
	print "loading biases from file", defBiases
	data = readfile(defBiases,0)
	size = len(data)
	biasNames = ['syth','syta','sypa','syph','tpb','cdr','cds','cdp','rpx','rpy','ifr','ift','ifl','cdof','sypw','syw','cdon','cdd','emch','emct','cdi','cdrg','self','foll','arbp','emvl','cdc','emvh']
	biasValues = zeros(size,int)
	sCmdTemp = Template("pythonInterface --rpcport /asvGrabber --request 'command:set,command:$biasName,int:$biasValue;'")

	for i in range (0,size):
		biasVal = data[i].rsplit("\n") #removes \n
		biasValues[i] = biasVal[0]
		sCmd = sCmdTemp.substitute(biasName = biasNames[i],biasValue = biasValues[i])
		os.system(sCmd)
	
	# program biases
	progBiases()

# --------------------------------------------- #
def asvDump(fileName,nSec):
	# dump data on file "fileName" (just fileName without path) for nSec seconds
	sCmdFileTemp = Template("pythonInterface --rpcport /asvGrabber --request 'command:dump,command:on,string:$fName;'")
	sCmdFile = sCmdFileTemp.substitute(fName = fileName)
	os.system(sCmdFile)
	time.sleep(nSec)
	os.system("pythonInterface --rpcport /asvGrabber --request 'command:dump,command:off;'")
	# move file from default directory to asv/dump directory
	os.system("mv /usr/local/src/robot/iCub/app/eMorphApplication/conf/asv*.txt /home/icub/asv/dump/")

	print 'data dumped on', fileName

