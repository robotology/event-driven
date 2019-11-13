# -*- coding: utf-8 -*-
'''
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotDvsLastTs takes a dict containing address-event data. 
Minimally, there must be x, y, and ts fields. 
For a given "time" (or a list of times, in which case subplots), 
creates a plot in which pixels are ordered according to their
last time stamp, and the order is represented by colour from red (recent)
to blue (oldest) through the spectrum.

Parameters which can be used:
 - time
 - minX
 - minY
 - maxX
 - maxY
 - flipVertical, flipHorizontal, transpose (applied in this order)
 - title
 - axis (to plot on; if not passed in, a new figure is created)
 - cmap
'''

import numpy as np
import matplotlib.pyplot as plt
from math import log10, floor
from scipy.stats import rankdata
import matplotlib.colors as colors

def roundToSf(x, sig=3): # https://stackoverflow.com/questions/3410976/how-to-round-a-number-to-significant-figures-in-python
    try:
        return round(x, sig-int(floor(log10(abs(x))))-1)
    except ValueError: # log of zero
        return 0

def plotDvsLastTsSingle(inDict, **kwargs):
    
    time = kwargs.get('time', kwargs.get('maxTime', np.max(inDict['ts'])))
    minTime = kwargs.get('minTime', np.min(inDict['ts']))
        
    # TODO: if the actual sensor size is known, use this instead of the following 
    minY = kwargs.get('minY',inDict['y'].min())
    maxY = kwargs.get('maxY',inDict['y'].max())
    minX = kwargs.get('minX',inDict['x'].min())
    maxX = kwargs.get('maxX',inDict['x'].max())
    sizeX = maxX - minX + 1
    sizeY = maxY - minY + 1
    tsArray = np.ones((sizeY, sizeX), dtype=np.float64) * -1

    # populate the array by running time forward to time
    chosenLogical = inDict['ts'] <= time
    chosenLogical &= inDict['x'] >= minX
    chosenLogical &= inDict['x'] <= maxX
    chosenLogical &= inDict['y'] >= minY
    chosenLogical &= inDict['y'] <= maxY
    xChosen = inDict['x'][chosenLogical] - minX
    yChosen = inDict['y'][chosenLogical] - minY
    tsChosen = inDict['ts'][chosenLogical]
    for x, y, ts in zip(xChosen, yChosen, tsChosen):
        tsArray[y, x] = ts         
    
    ordinal = kwargs.get('ordinal', False)
    if ordinal:
        tsArrayFlattened = tsArray.flatten()
        tsOrdinal = rankdata(tsArrayFlattened, method='dense') - 1 # min rank is 1
        if np.any(tsArrayFlattened == -1):
            tsOrdinal -= 1 # If there are unset timestamps, they will have rank 0 - push these to -1
        tsArray = tsOrdinal.reshape(tsArray.shape)
        
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    transpose = kwargs.get('transpose', False)
    if transpose:
        tsArray = np.transpose(tsArray)
    cmap = plt.get_cmap(kwargs.get('cmap', 'jet'))
    cmap.set_under(color='white')
    image = axes.imshow(tsArray, origin='lower', cmap=cmap, norm=colors.Normalize(vmin=0, vmax=np.max(tsArray)))
    axes.set_aspect('equal', adjustable='box')
    if kwargs.get('flipVertical', False):
        axes.invert_yaxis()
    if kwargs.get('flipHorizontal', False):
        axes.invert_xaxis()
    title = kwargs.get('title')
    if title is not None:
        axes.set_title(title)
    axes.set_title(str(roundToSf(minTime)) + ' - ' + str(roundToSf(time)) + ' s')
    
    callback = kwargs.get('callback')
    if callback is not None:
        callback(tsArray=tsArray, **kwargs)
    
    return image

def plotDvsLastTs(inDict, **kwargs):
    times = kwargs.get('time', kwargs.get('maxTime', np.max(inDict['ts'])))
    if np.isscalar(times):
        times = [times]
    numPlots = len(times)
    numPlotsX = int(round(np.sqrt(numPlots / 3 * 4)))
    numPlotsY = int(np.ceil(numPlots / numPlotsX))
    fig, allAxes = plt.subplots(numPlotsY, numPlotsX)
    if numPlots == 1:
        allAxes = [allAxes]
    else:
        allAxes = allAxes.flatten()
    fig.suptitle(kwargs.get('title', ''))
    for time, axes in zip(times, allAxes):
        kwargs['time'] = time
        kwargs['axes'] = axes
        plotDvsLastTsSingle(inDict, **kwargs)
