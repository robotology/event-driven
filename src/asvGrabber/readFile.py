#!/usr/bin/python 

import sys

def readfile(filename):
	'''Print a file to the standard output.'''
	f = file(filename)
	while True:
		line = f.readline()
		if len(line) == 0:
			break
		print line, # notice comma
	f.close()
