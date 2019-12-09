# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
plotCorrelogram receives "inDict", which is a container for (2D) polarity events, 
(Or a higher level container), containing fields with numpy arrays:    
    ts
    x
    y
    pol
Autocorrelation
A correlogram is created for the chosen range of pixels.
Choose the pixels by setting the kwargs:
    xMin
    xMax
    yMin
    yMax
By default, all pixels are included. 
By default, all pixels are autocrrelated individually, and then the results are 
combined, so the end result is a mean over all pixels.
TODO: Could do an autocrrelation for all spikes for the range together.
TODO: Could write a cross-correlation function.
minTime and maxTime are used to constrain time. 
By default, polarities are combined; separate them with splitByPol=true
"""

#%% Plot tsPix (single pixel timestamps)

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import numpy as np
from tqdm import tqdm

# local imports

def plotAutoCorrelation(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotCorrelogram(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotDvs was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotCorrelogram(channelData['dvs'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')
        return
    # use other library functions to do spatiotemporal cropping
    inDict = cropSpace(inDict, **kwargs)
    inDict = cropTime(inDict, zeroTime=False, **kwargs)
    if kwargs.get(splitByPol, False):
        splitDict = splitByPolarity(inDict)
        fig, axes = plt.subplots(2, 1)
        kwargs['axes'] = axes[0]
        plotAutoCorrelation(splitDict[0], **kwargs)
        kwargs['axes'] = axes[0]
        plotAutoCorrelation(splitDict[1], **kwargs)
        return
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes
    # Break out data arrays for cleaner code
    x = inDict['x']
    y = inDict['y']
    ts = inDict['ts']
    pol = inDict['pol']
    numBins = kwargs.get('numBins', 50)
    minOffset = kwargs.get('minOffset', 0.00001) # 10 us
    maxOffset = kwargs.get('maxOffset', 0.1) # 100 ms
    boundaries = np.logspace(minOffset, maxOffset, numBins)
    # Cropping has already happened; now iterate through defacto spatial range
    for currX in range(min(x), max(x)+1): 
        for currY in range(min(y), max(y)+1): 
    histPos = np.zeros()


    axes.add_line(line)
    axes.set_xlim(minTime-timeRange*0.01, maxTime+timeRange*0.01)
    axes.set_ylim(minX + minY*numX - 1, maxX + maxY*numX + 1)
    formatString = '0'+ str(int(np.log10(999))+1) + 'd'
    theRange = range(minX+minY*numX, maxX+maxY*numX)
    labels = ['x' + format(np.mod(x,numX), formatString) + ',' +
              'y' + format(int(x/numX), formatString)
              for x in theRange]
    plt.yticks(theRange, labels)
    
    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs)

# For now, this is synonymous with autocrrelation
# TODO: cross-correlation could be useful
def plotCorrelogram(inDict, **kwargs):
