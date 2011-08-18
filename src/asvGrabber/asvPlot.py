#!/usr/bin/env python

import sys
#import numpy as np
from numpy import *
import matplotlib as mplot
#import mplot.pyplot as pplot
#import matplotlib.pyplot as plt
import pylab as p
#from mpl_toolkits.axes_grid1 import host_subplot


data = sys.stdin.readlines()
print "Counted", len(data), "lines."
x = array( [0,1,2,3,4,5,6,7,8,9])
y = array( [0,1,2,3,4,5,6,7,8,9])
print x
print y
#ax = host_subplot(111, axes_class=AA.Axes)#
#fig = plt.figure()
#ax = fig.add_subplot(111)

#mplot.pyplot.plot(x,y)
#pylab.show()
#plt.draw()
#plt.show()
