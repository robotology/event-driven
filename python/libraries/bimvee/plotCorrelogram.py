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
By default, all pixels are autocorrelated individually, and then the results are 
combined, so the end result is a mean over all pixels.
TODO: Could do an autocorrelation for all spikes for the range together.
minTime and maxTime are used to constrain time. 
By default, polarities are combined; separate them with splitByPol=true
numBins is interpretted as the number of time bins in each of positive 
and negative directions, not including a central bin. 
"""

#%% Plot tsPix (single pixel timestamps)

import matplotlib.pyplot as plt
import numpy as np
from tqdm import trange

# local imports
if __package__ is None or __package__ == '':
    from split import cropSpace, cropTime, splitByPolarity
else:
    from .split import cropSpace, cropTime, splitByPolarity
    
# TODO: include a linear spacing option
def defineBoundariesAndDensities(**kwargs):
    numBins = kwargs.get('numBins', 50)
    minOffset = kwargs.get('minOffset', 0.00001) # 10 us
    maxOffset = kwargs.get('maxOffset', 1.0) # 100 ms
    logSpacing = kwargs.get('spacing', 'linear') == 'log'
    mirrored = kwargs.get('mirrored', True)
    if logSpacing:
        boundaries = np.geomspace(minOffset, maxOffset, numBins+1)
        if mirrored:
            boundaries = np.concatenate((-np.flip(boundaries), boundaries))
        elif not mirrored:
            boundaries = np.concatenate((np.zeros((1)), boundaries))        
    elif (not logSpacing) and mirrored: # Linear; ignore minOffset
        boundaries = np.linspace(-maxOffset, maxOffset, numBins*2)
    elif (not logSpacing) and (not mirrored):
        boundaries = np.linspace(0, maxOffset, (numBins))
    # now there are boundaries either starting from zero ot mirrored around zero 
    # for each bin, including a central bin, and the extreme boundaries 
    # are the edge of the time region of interest.
    widths = (boundaries[1:] - boundaries[:-1])
    densities = 1 / widths
    centres = (boundaries[1:] + boundaries[:-1]) / 2
    return boundaries, densities, centres, widths

def interSpikeInterval(ts, boundaries, densities, **kwargs):
    hist = np.zeros((len(densities)), dtype=np.float64)
    diff = ts[1:] - ts[:-1]
    for idx, (boundary1, boundary2) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        hist[idx] = np.sum(np.logical_and(diff >= boundary1, diff <= boundary2))
    hist = hist * densities
    sumHist = np.sum(hist) # Avoid dividing by zero in case the hist is empty
    if sumHist:
        hist = hist / sumHist
    return hist

# Plot the timings of the spikes in events2 w.r.t. the spikes in events1
def plotInterSpikeIntervalSingle(events, **kwargs):    
    if (len(events['ts']) == 0):
        return
    # use other library functions to do spatiotemporal cropping
    # If this is called from plotInterSpikeInterval then this has already been done, 
    # But do it here as well so that this function can be independent.
    events = cropSpace(events, **kwargs)
    events = cropTime(events, zeroTime=False, **kwargs)
    kwargs['mirrored'] = False
    boundaries, densities, centres, widths = defineBoundariesAndDensities(**kwargs)
    # Spatial cropping has happened; now iterate through defacto spatial range
    minX = np.min(events['x'])
    maxX = np.max(events['x'])
    minY = np.min(events['y'])
    maxY = np.max(events['y'])
    hist = np.zeros((len(densities)), dtype=np.float64)
    for currX in trange(minX, maxX+1, leave=True, position=0):
        currXLogical = events['x']==currX
        tsForCurrX = events['ts'][currXLogical]
        yForCurrX = events['y'][currXLogical]
        for currY in range(minY, maxY+1):
            tsXY = tsForCurrX[yForCurrX==currY]
            if np.any(tsXY): 
                hist = hist + interSpikeInterval(tsXY, boundaries, densities, **kwargs)
    # Normalise
    weightedSum = np.sum(hist / densities)
    if weightedSum > 0:
        hist = hist / np.sum(weightedSum)
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    axes.bar(centres, hist, widths, antialiased=False)
    #axes.set_ylim(minX + minY*numX - 1, maxX + maxY*numX + 1)
    #plt.xticks(theRange, labels)
    title = kwargs.get('title', '')
    if kwargs.get('minX', False) or  kwargs.get('maxX', False):
        title = title + ' ' + str(minX) + '<=X<=' + str(maxX)
    if kwargs.get('minY', False) or  kwargs.get('maxY', False):
        title = title + ' ' + str(minY) + '<=Y<=' + str(maxY)
    axes.set_title(title)

    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs) 
        
def plotInterSpikeInterval(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotInterSpikeInterval(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotInterSpikeInterval was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotInterSpikeInterval(channelData['dvs'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')
        return
    print('plotInterSpikeInterval working: ' + kwargs.get('title', 'unnamed'))
    # use other library functions to do spatiotemporal cropping
    inDict = cropSpace(inDict, **kwargs)
    inDict = cropTime(inDict, zeroTime=False, **kwargs)
    if kwargs.get('splitByPol', False):
        splitDict = splitByPolarity(inDict)        
        fig, axes = plt.subplots(1, 3)

        kwargs['axes'] = axes[0]
        kwargs['title'] = 'on + off'
        print(kwargs['title'])
        plotInterSpikeIntervalSingle(inDict, **kwargs)

        kwargs['axes'] = axes[1]
        kwargs['title'] = 'off'
        print(kwargs['title'])
        plotInterSpikeIntervalSingle(splitDict['0'], **kwargs)

        kwargs['axes'] = axes[2]
        kwargs['title'] = 'on'
        print(kwargs['title'])
        plotInterSpikeIntervalSingle(splitDict['1'], **kwargs)

    else:            
        axes = kwargs.get('axes')
        if axes is None:
            fig, axes = plt.subplots()
            kwargs['axes'] = axes
        kwargs['title'] = 'auto'
        plotInterSpikeIntervalSingle(inDict, **kwargs)
    
# Alias using common abbreviation
def plotIsi(inDict, **kwargs):
    plotInterSpikeInterval(inDict, **kwargs)

def crossCorrelation(ts1, ts2, boundaries, densities, **kwargs):
    hist = np.zeros((len(densities)), dtype=np.float64)
    # TODO: think about a nice array-based way to do the following:
    for ts in ts1:
        firstIdx = np.searchsorted(ts2, ts+boundaries[0])
        lastIdx = np.searchsorted(ts2, ts+boundaries[-1])
        selectedTs2 = ts2[firstIdx: lastIdx] - ts
        histIds = np.searchsorted(boundaries, selectedTs2) - 1
        for histIdx in histIds:
            try:
                hist[histIdx] = hist[histIdx] + 1
            except IndexError:
                pass # Out of bounds - we don't want it
    # If this was intended as an autocorrelation then we need to remove the 
    # self-correlations
    if kwargs.get('auto', False):
        centreIdx = int(len(boundaries) / 2) - 1 
        hist[centreIdx] = hist[centreIdx] - len(ts1)
    hist = hist * densities
    sumHist = np.sum(hist) # Avoid dividing by zero in case the hist is empty
    if sumHist:
        hist = hist / sumHist
    return hist

# Plot the timings of the spikes in events2 w.r.t. the spikes in events1
def plotCrossCorrelation(events1, events2, **kwargs):    
    if (len(events1['ts']) == 0) or (len(events2['ts']) == 0):
        return
    # use other library functions to do spatiotemporal cropping
    # If this is called from plotautocorrelation then this has already been done, 
    # But do it here as well so that this function can be independent.
    events1 = cropSpace(events1, **kwargs)
    events1 = cropTime(events1, zeroTime=False, **kwargs)
    events2 = cropSpace(events2, **kwargs)
    events2 = cropTime(events2, zeroTime=False, **kwargs)
    boundaries, densities, centres, widths = defineBoundariesAndDensities(**kwargs)
    # Spatial cropping has happened; now iterate through defacto spatial range
    minX = min(np.min(events1['x']), np.min(events2['x']))
    maxX = max(np.max(events1['x']), np.max(events2['x']))
    minY = min(np.min(events1['y']), np.min(events2['y']))
    maxY = max(np.max(events1['y']), np.max(events2['y']))
    hist = np.zeros((len(densities)), dtype=np.float64)
    # TODO: This loop could be much more efficient, following the pattern in plotInterSpikeInterval, above
    for currX in trange(minX, maxX+1, leave=True, position=0): 
        for currY in range(minY, maxY+1):
            ts1 = events1['ts'][np.logical_and(events1['x']==currX, events1['y']==currY)]
            ts2 = events2['ts'][np.logical_and(events2['x']==currX, events2['y']==currY)]
            if np.any(ts1) and np.any(ts2): 
                hist = hist + crossCorrelation(ts1, ts2, boundaries, densities, **kwargs)
    # Normalise
    weightedSum = np.sum(hist / densities)
    if weightedSum > 0:
        hist = hist / np.sum(weightedSum)
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes

    axes.bar(centres, hist, widths, antialiased=False)
    #axes.set_ylim(minX + minY*numX - 1, maxX + maxY*numX + 1)
    #plt.xticks(theRange, labels)
    title = kwargs.get('title', '')
    if kwargs.get('minX', False) or  kwargs.get('maxX', False):
        title = title + ' ' + str(minX) + '<=X<=' + str(maxX)
    if kwargs.get('minY', False) or  kwargs.get('maxY', False):
        title = title + ' ' + str(minY) + '<=Y<=' + str(maxY)
    axes.set_title(title)

    callback = kwargs.get('callback')
    if callback is not None:
        callback(**kwargs)    

def plotAutoCorrelation(inDict, **kwargs):
    # Boilerplate for descending container hierarchy
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotCorrelogram(inDictInst, **kwargs)
        return
    if 'info' in inDict: # Top level container
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotAutocorrelation was called for file ' + fileName)
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
    print('plotAutocorrelation working: ' + kwargs.get('title', 'unnamed'))
    # use other library functions to do spatiotemporal cropping
    inDict = cropSpace(inDict, **kwargs)
    inDict = cropTime(inDict, zeroTime=False, **kwargs)
    if kwargs.get('splitByPol', False):
        splitDict = splitByPolarity(inDict)        
        fig, axes = plt.subplots(2, 2)

        kwargs['axes'] = axes[0, 0]
        kwargs['title'] = 'on + off auto'
        print(kwargs['title'])
        kwargs['auto'] = True
        plotCrossCorrelation(inDict, inDict, **kwargs)

        kwargs['axes'] = axes[0, 1]
        kwargs['title'] = 'off auto'
        print(kwargs['title'])
        kwargs['auto'] = True
        plotCrossCorrelation(splitDict['0'], splitDict['0'], **kwargs)

        kwargs['axes'] = axes[1, 0]
        kwargs['title'] = 'on auto'
        print(kwargs['title'])
        kwargs['auto'] = True
        plotCrossCorrelation(splitDict['1'], splitDict['1'], **kwargs)

        kwargs['axes'] = axes[1, 1]
        kwargs['title'] = 'on w.r.t. off cross'
        print(kwargs['title'])
        kwargs['auto'] = False
        plotCrossCorrelation(splitDict['0'], splitDict['1'], **kwargs)

    elif kwargs.get('splitByPolAlt', False):
        splitDict = splitByPolarity(inDict)        
        fig, axes = plt.subplots(3, 1)
        kwargs['axes'] = axes[0]
        kwargs['title'] = 'on + off auto'
        print(kwargs['title'])
        kwargs['auto'] = True
        plotCrossCorrelation(inDict, inDict, **kwargs)
        kwargs['axes'] = axes[1]
        kwargs['title'] = 'on w.r.t. off cross'
        print(kwargs['title'])
        kwargs['auto'] = False
        plotCrossCorrelation(splitDict['0'], splitDict['1'], **kwargs)
        kwargs['axes'] = axes[2]
        kwargs['title'] = 'off w.r.t. on cross'
        print(kwargs['title'])
        kwargs['auto'] = False
        plotCrossCorrelation(splitDict['1'], splitDict['0'], **kwargs)
    else:            
        axes = kwargs.get('axes')
        if axes is None:
            fig, axes = plt.subplots()
            kwargs['axes'] = axes
        kwargs['title'] = 'auto'
        kwargs['auto'] = True
        plotCrossCorrelation(inDict, inDict, **kwargs)

# For a single trace, this is synonymous with autocorrelation
def plotCorrelogram(inDict, **kwargs):
    plotAutoCorrelation(inDict, **kwargs)