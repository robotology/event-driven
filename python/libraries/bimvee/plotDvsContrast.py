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
plotDvsContrast takes 'inDict' - a dict containing an imported ae file, 
as created by importAe, and creates a series of green/red plots of
polarity data.
It creates a completely separate plot for each channel which contains 'pol' data
Creates an image from events with contrast, by accumulating events up and down 
to a maximum level at which full color is used. 
The number of subplots is given by the numPlots parameter.
'distributeBy' can either be 'time' or 'events', to decide how the points 
around which data is rendered are chosen.
The events are then recruited by the time points, spreading out until
either they are about to overlap with a neighbouring point, or until 
a certain ratio of a full array is reached. 

Parameters which can be used:
 - numPlots
 - distributeBy
 - minTime
 - maxTime
 - proportionOfPixels
 - contrast
 - flipVertical
 - flipHorizontal
 - transpose
'''

import numpy as np
import matplotlib.pyplot as plt
from math import log10, floor
from plotDvsLastTs import plotDvsLastTs

def roundToSf(x, sig=3):
    try:
        return round(x, sig-int(floor(log10(abs(x))))-1)
    except ValueError: # log of zero
        return 0

'''
nomenclature:
    idx = index
    ids = indices
'''
def idsEventsInTimeRange(events, **kwargs):
    startTime = kwargs.get('startTime', events['ts'][0])
    endTime = kwargs.get('endTime', events['ts'][-1])
    # The following returns logical indices
    #return (events['ts'] >= startTime) & (events['ts'] < endTime)
    # Alternatively, search for the start and end indices, then return a range
    # This might be faster, given that the ts array is already sorted
    startIdx = np.searchsorted(events['ts'], startTime)
    endIdx = np.searchsorted(events['ts'], endTime)
    return range(startIdx, endIdx)

def getEventImage(events, **kwargs):
    ids = kwargs.get('ids', idsEventsInTimeRange(events, **kwargs))
    # dims might be in the events dict, but allow override from kwargs
    dimX = kwargs.get('dimX', events.get('dimX', np.max(events['x'])+1))
    dimY = kwargs.get('dimY', events.get('dimY', np.max(events['y'])+1))
    eventImage = np.histogram2d(events['y'][ids], 
                                 events['x'][ids], 
                                 bins=[dimY, dimX],
                                 range=[[0, dimY-1], [0, dimX-1]]
                                 )[0]
    if kwargs.get('contrast') is not None:
        eventImage = np.clip(eventImage, 0, kwargs.get('contrast'))
    return eventImage

'''
TODO: apply the low-level functions above into the plot functions below
'''
def plotDvsContrastSingle(inDict, **kwargs):
    # Unpack data for clarity
    pol = inDict['pol']
    x = inDict['x']
    y = inDict['y']
    # Accumulate x and y separately for pos and neg pol, then subtract one from the other.
    xPos = x[pol]
    yPos = y[pol]
    maxX = kwargs.get('maxX', inDict['x'].max())
    maxY = kwargs.get('maxY', inDict['y'].max())
    frameFromEventsPos = np.histogram2d(yPos, xPos, bins=[maxY, maxX], range=[[0, maxY-1], [0, maxX-1]])[0]
    xNeg = x[~pol]
    yNeg = y[~pol]
    frameFromEventsNeg = np.histogram2d(yNeg, xNeg, bins=[maxY, maxX], range=[[0, maxY-1], [0, maxX-1]])[0]
    frameFromEvents = frameFromEventsPos - frameFromEventsNeg
    if kwargs.get('transpose', False):
        frameFromEvents = np.transpose(frameFromEvents)
    # The 'contrast' for display of events, as used in jAER.
    contrast = kwargs.get('contrast', 3)
    # Clip the values according to the contrast
    frameFromEvents = np.clip(frameFromEvents, -contrast, contrast)
    #frameFromEvents = frameFromEvents + contrast
    cmap = kwargs.get('cmap', kwargs.get('colormap', 'seismic_r'))
    
    axes = kwargs.get('axes')
    if axes is None:
        fig, axes = plt.subplots()
    image = axes.imshow(frameFromEvents, origin='lower', cmap=cmap, 
                        vmin=-contrast, vmax=contrast)
    axes.set_aspect('equal', adjustable='box')
    if kwargs.get('flipVertical', False):
        axes.invert_yaxis()
    if kwargs.get('flipHorizontal', False):
        axes.invert_xaxis()
    title = kwargs.get('title')
    if title is not None:
        axes.set_title(title)
    
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = axes
        callback(frameFromEvents=frameFromEvents, **kwargs)
    return image

def plotDvsContrast(inDict, **kwargs):
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotDvsContrast(inDictInst, **kwargs)
        return
    if 'info' in inDict:
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotDvs was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotDvsContrast(channelData['dvs'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')
        return
    # The proportion of an array-full of events which is shown on a plot
    proportionOfPixels = kwargs.get('proportionOfPixels', 0.1)
    # useAllData overrides the above - the windows used for the images together include all the data 
    useAllData = kwargs.get('useAllData', False)
    
    numPlots = kwargs.get('numPlots', 6)

    # Choice of distributing by 'time' or by 'numEvents'
    distributeBy = kwargs.get('distributeBy', 'time').lower()
    
   #% Distribute plots in a raster with a 3:4 ratio
    numPlotsX = int(round(np.sqrt(numPlots / 3 * 4)))
    numPlotsY = int(np.ceil(numPlots / numPlotsX))
    
    # TODO: if the actual sensor size is known, use this instead of the following defaults
    minX = kwargs.get('minX', inDict['x'].min())
    maxX = kwargs.get('maxX', inDict['x'].max())
    minY = kwargs.get('minY', inDict['y'].min())
    maxY = kwargs.get('maxY', inDict['y'].max())
    
    numPixelsInArray = (maxY + 1 - minY) * (maxX + 1 - minX)
    numEventsToSelectEachWay = int(round(numPixelsInArray * proportionOfPixels / 2.0))

    # The following section results in a set of tuples of first and last event idx
    # one for each plot. It does this considering the choices of distributeBy, useAllData, and proportionOfPixels
    
    #unpack ts for brevity
    ts = inDict['ts']
    
    minTime = kwargs.get('minTime', ts.min())
    maxTime = kwargs.get('maxTime', ts.max())
    minEventIdx = np.searchsorted(ts, minTime)
    maxEventIdx = np.searchsorted(ts, maxTime)
    numEvents = maxEventIdx-minEventIdx
    if distributeBy == 'time':
        totalTime = maxTime - minTime
        timeStep = totalTime / numPlots
        if useAllData:
            timeBoundaries = np.arange(minTime, maxTime + timeStep / 2, timeStep)
            firstEventIds = [
                np.where(ts >= timeBoundary)[0][0]
                for timeBoundary in timeBoundaries ]
            lastEventIds = [firstEventIdx - 1 for firstEventIdx in firstEventIds]
            lastEventIds = lastEventIds[1:]
            firstEventIds = firstEventIds[:-1]
        else:
            timeCentres = np.arange(minTime + timeStep * 0.5, maxTime, timeStep)
            centreEventIds = [ 
                np.where(ts >= timeCentre)[0][0]
                for timeCentre in timeCentres ]
            firstEventIds = [idx - numEventsToSelectEachWay for idx in centreEventIds]
            lastEventIds = [idx + numEventsToSelectEachWay for idx in centreEventIds]
    else: # distribute by event number
        eventsPerStep = int(numEvents / numPlots)
        if useAllData:
            firstEventIds = range(minEventIdx, maxEventIdx, eventsPerStep)
            lastEventIds = [firstEventIdx - 1 for firstEventIdx in firstEventIds]
            lastEventIds = lastEventIds[1:]
            firstEventIds = firstEventIds[:-1]
        else:
            centreEventIds = range(int(eventsPerStep/2), numEvents, eventsPerStep)
            firstEventIds = [idx - numEventsToSelectEachWay for idx in centreEventIds]
            lastEventIds = [idx + numEventsToSelectEachWay for idx in centreEventIds]
    firstEventIds = np.clip(firstEventIds, 0, numEvents - 1)
    lastEventIds = np.clip(lastEventIds, 0, numEvents)
    firstTimes = ts[firstEventIds]
    lastTimes = ts[lastEventIds]
    timeCentres = firstTimes+lastTimes/2
    titles = [
        str(roundToSf(firstTime)) + ' - ' + str(roundToSf(lastTime)) + ' s'
        for firstTime, lastTime in zip(firstTimes, lastTimes) ]

    fig, allAxes = plt.subplots(numPlotsY, numPlotsX)
    if numPlots == 1:
        allAxes = [allAxes]
    else:
        allAxes = allAxes.flatten()
    fig.suptitle(kwargs.get('title', ''))
    
    for axes, firstEventIdx, lastEventIdx, title in zip(allAxes, firstEventIds, lastEventIds, titles):
        dvsDataDict = {
            'x': inDict['x'][firstEventIdx:lastEventIdx],
            'y': inDict['y'][firstEventIdx:lastEventIdx],
            'pol': inDict['pol'][firstEventIdx:lastEventIdx],
            'ts': inDict['pol'][firstEventIdx:lastEventIdx]
                }
        kwargs['title'] = title
        image = plotDvsContrastSingle(inDict=dvsDataDict, axes=axes, **kwargs)
    fig.colorbar(image)
    return timeCentres

'''
Takes an importedDict or a list of them
For each channel in each, which contains polarity data , call PlotPolarity
'''

def plotDvsContrastForImportedDicts(inDicts, **kwargs):
    if not isinstance(inDicts, list):
        inDicts = [inDicts]
    for inDict in inDicts:
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotDvs was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotDvsContrast(channelData['dvs'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')

''' generalised plot function for a container at multiple levels '''
def plotDvs(inDicts, **kwargs):
    plotType = kwargs.get('plotType', 'contrast')
    if plotType == 'lastTs':
        plotDvsLastTs(inDicts, **kwargs)
        # TODO: branch off other plot types here
    else:
        plotDvsContrast(inDicts, **kwargs)
                    