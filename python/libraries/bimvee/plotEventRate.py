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
plotEventRate takes 'inDict' - a dict containing an imported ae file, 
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

def roundToSf(x, sig=3):
    try:
        return round(x, sig-int(floor(log10(abs(x))))-1)
    except ValueError: # log of zero
        return 0
'''
def plotEventRate(inDict, **kwargs):
    # Break out data arrays for cleaner code
    #x = inDict['x']
    #y = inDict['y']
    ts = inDict['ts']
    #pol = inDict['pol']
    startTime = kwargs.get('startTime', np.min(ts))
    endTime = kwargs.get('endTime', np.max(ts))
    freqs = kwargs.get('freqs')
    if freqs is None:
        periods = kwargs.get('periods', [0.001, 0.01, 0.1, 1])
    else:
        periods = [1/f for f in freqs]

    axes = kwargs.get('axes') 
    if axes is None:
        fig, axes = plt.subplots()
        kwargs['axes'] = axes
    legend = kwargs.get('legend', [])
    for period in periods:
        endTimes = [t + period for t in np.arange(startTime, endTime, period)]
        midTimes = [t - period / 2 for t in endTimes]
        endIds = [np.searchsorted(ts, t) for t in endTimes]
        counts = [end-start for start, end in zip(endIds[:-1], endIds[1:])]
        counts.insert(0, endIds[0])
        rates = [count/period for count in counts]
        endTimes = np.arange(startTime, endTime, period)
        axes.plot(midTimes, rates)
        plt.xlabel('Time (s)')
        plt.ylabel('Rate (events/s)')
        legend.append('period: ' + str(period) + ' s')
    axes.legend(legend)
    kwargs['legend'] = legend
    if kwargs.get('title') is not None:
        axes.set_title(kwargs.get('title'))
    
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs = callback(**kwargs)
    
    return kwargs
'''
def plotEventRate(inDicts, **kwargs):
    if not isinstance(inDicts, list):
        inDicts = [inDicts]
    for inDict in inDicts:
        if 'ts' in inDict: # It's a data-type container
            # Break out data array for cleaner code
            ts = inDict['ts']
            startTime = kwargs.get('startTime', np.min(ts))
            endTime = kwargs.get('endTime', np.max(ts))
            freqs = kwargs.get('freqs')
            if freqs is None:
                periods = kwargs.get('periods', [0.001, 0.01, 0.1, 1])
            else:
                periods = [1/f for f in freqs]        
            axes = kwargs.get('axes') 
            if axes is None:
                fig, axes = plt.subplots()
                kwargs['axes'] = axes
            legend = kwargs.get('legend', [])
            for period in periods:
                endTimes = [t + period for t in np.arange(startTime, endTime, period)]
                midTimes = [t - period / 2 for t in endTimes]
                endIds = [np.searchsorted(ts, t) for t in endTimes]
                counts = [end-start for start, end in zip(endIds[:-1], endIds[1:])]
                counts.insert(0, endIds[0])
                rates = [count/period for count in counts]
                endTimes = np.arange(startTime, endTime, period)
                axes.plot(midTimes, rates)
                plt.xlabel('Time (s)')
                plt.ylabel('Rate (events/s)')
                legend.append('period: ' + str(period) + ' s')
            axes.legend(legend)
            kwargs['legend'] = legend
            if kwargs.get('title') is not None:
                axes.set_title(kwargs.get('title'))            
            callback = kwargs.get('callback')
            if callback is not None:
                kwargs = callback(**kwargs)            
        else:
            # Is it a top-level container?
            if 'info' not in inDict:
                return kwargs
            fileName = inDict['info'].get('filePathOrName', '')
            print('plotEventRate was called for file ' + fileName)
            if not inDict['data']:
                print('The import contains no data.')
                return
            for channelName in inDict['data'].keys():
                channelData = inDict['data'][channelName]
                if 'dvs' in channelData and len(channelData['dvs']['ts']) > 0:
                    kwargs['title'] = ' '.join([fileName, str(channelName)])
                    plotEventRate(channelData['dvs'], **kwargs)
                else:
                    print('Channel ' + channelName + ' skipped because it contains no polarity data')
    return kwargs

                
                