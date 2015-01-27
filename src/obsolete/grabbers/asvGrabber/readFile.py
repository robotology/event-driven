#!/usr/bin/python 

import sys
import array

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
		#print line,buffer     # notice comma
	f.close()
	return arr



buffer = readfile("/home/icub/asv/dump/asvTOT.txt", 0)
