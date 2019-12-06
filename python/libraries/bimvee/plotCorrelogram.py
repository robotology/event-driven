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
A correlogram is created for a has a row for each address-event represented, where the x dim 
represents time. Color represents polarity (blue vs red for 1 vs 0)
By default, all addresses will be plotted as a row, 
but this will usually be impractical. So the kwargs
    xMin
    xMax
    yMin
    yMax
are used to constrain the space represented
and tMin and tMax are used to constrain time. 
The axes are aligned so that the coordinate points to the middle of the first
row represented by that mark. 

"""

#%% Plot tsPix (single pixel timestamps)

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import numpy as np
from tqdm import tqdm

def plotAutoCorrelation():
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
    # Break out data arrays for cleaner code
    x = inDict['x']
    y = inDict['y']
    ts = inDict['ts']
    pol = inDict['pol']
    minX = kwargs.get('minX', np.min(x))
    maxX = kwargs.get('maxX', np.max(x))
    minY = kwargs.get('minY', np.min(y))
    maxY = kwargs.get('maxY', np.max(y))
    minTime = kwargs.get('minTime', np.min(ts))
    maxTime = kwargs.get('maxTime', np.max(ts))
    numX = maxX - minX + 1
    #numY = maxY - minY + 1
    timeRange = maxTime - minTime
    selected = np.where((x >= minX) & (x <= maxX) &
                    (y >= minY) & (y <= maxY) &
                    (ts >= minTime) & (ts <= maxTime))[0]

    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    for idx in tqdm(selected):
        xPos = ts[idx]
        yPos = x[idx] + y[idx]*numX
        if pol[idx]:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='blue', lw=2, axes=axes)
        else:
            line = lines.Line2D([xPos, xPos], [yPos - 0.5, yPos + 0.5], 
                                color='red', lw=2, axes=axes)
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
