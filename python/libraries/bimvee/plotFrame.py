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
plotFrame takes 'inDict' - a dictionary containing imported frame data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and creates a series of images from selected
frames.
The number of subplots is given by the numPlots parameter.
'distributeBy' can either be 'time' or 'events', to decide how the points 
around which data is rendered are chosen. 
The frame events are then chosen as those nearest to the time points.
If the 'distributeBy' is 'time' then if the further parameters 'minTime' 
and 'maxTime' are used then the time window used is only between
those limits.
Params include:
numPlots, distributeBy, minTime, maxTime, flipVertical, flipHorizontal, transpose
'''

import numpy as np
import matplotlib.pyplot as plt
from math import log10, floor

def roundToSf(x, sig=3):
    try:
        return round(x, sig-int(floor(log10(abs(x))))-1)
    except ValueError: # log of zero
        return 0

def plotFrame(inDict, **kwargs):
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotFrame(inDictInst, **kwargs)
        return
    if 'info' in inDict:
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotFrame was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'frame' in channelData and len(channelData['frame']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotFrame(channelData['frame'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no frame data')
        return    
    distributeBy = kwargs.get('distributeBy', 'time').lower()
    numPlots = kwargs.get('numPlots', 6)
    
    ts = inDict['ts']
    frames = inDict['frames']
    numFrames = len(ts)
    if numFrames < numPlots:
        numPlots = numFrames

    if numFrames == numPlots:
        distributeBy = 'events'
    
    # Distribute plots in a raster with a 3:4 ratio
    numPlotsX = int(np.round(np.sqrt(numPlots / 3 * 4)))
    numPlotsY = int(np.ceil(numPlots / numPlotsX))
    
    minTime = kwargs.get('startTime', kwargs.get('minTime', kwargs.get('beginTime', ts[0])))
    maxTime = kwargs.get('stopTime', kwargs.get('maxTime', kwargs.get('endTime', ts[-1])))

    if distributeBy == 'time':
        totalTime = maxTime - minTime
        timeStep = totalTime / numPlots
        timePoints = np.arange(minTime + timeStep * 0.5, maxTime, timeStep)
    else: # distribute by event number
        framesPerStep = numFrames / numPlots
        timePoints = ts(int(np.ceil(np.arange(framesPerStep * 0.5, numFrames, framesPerStep))))

    fig, axes = plt.subplots(numPlotsY, numPlotsX)
    fig.suptitle(kwargs.get('title', ''))
    
    axes = axes.flatten().tolist()
    for ax, timePoint in zip(axes, timePoints):

        # Find eventIndex nearest to timePoint
        frameIdx = np.searchsorted(ts, timePoint)
        frame = frames[frameIdx]
        if kwargs.get('transpose', False):
            frame = np.transpose(frame)
        if kwargs.get('flipVertical', False):
            frame = np.flip(frame, axis=0)
        if kwargs.get('flipHorizontal', False):
            frame = np.flip(frame, axis=1)
        ax.imshow(frame, cmap='gray')
        ax.set_title('Time: ' + str(roundToSf(timePoint)) + ' s; frame number: ' + str(frameIdx))

#%%
'''
    Optional extra - not including it by default because I don't want this 
    extra dependency, but this is a quick way to see frame data as a video
'''

'''       
from imageio import mimwrite
def framesToGif(framesDict, **kwargs):
    ts = framesDict['ts']
    frames = framesDict['frames']
    outputFilePathAndName = kwargs.get('outputFilePathAndName', 'framesAsMovie.mp4')
    frameRate = len(ts) / (ts[-1] - ts[0])
    framesExpanded = [np.expand_dims(f, 0) for f in frames]
    framesAllArray = np.concatenate(framesExpanded, 0)
    mimwrite(outputFilePathAndName, framesAllArray , fps = int(frameRate))
'''

