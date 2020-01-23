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
Various functions for working with timestamps

zeroTimestamps(imports)

imports can either be an importDict as produced by importAe, or it can be a list of them. 
For each import, for each channel and for each datatype, the first timestamp is found. 
We assume that these timestamps are already synchronised.
We take the lowest, and use that to zero all the timestamps. 

unwrapTimestamps()
taking a numpy array (expected to by dtype=np.float64), return the array where
any timestamp wrap events have been treated by advancing all the timestamps 
after every wrap event.
The 'wrapTime' argument is the time at which wrapping is expected to occur.
If this is not given, then the maximum timestamp before each wrap event is used.
    
"""

#%%

import numpy as np

def zeroTimestampsForADataType(dataTypeDict, tsOffset=None):
    # Probably the common format is 'ts' for all dtypes, 
    # but handle any exceptions here, example: if dtypeName == 'frame':
    tsOffsetInitial = dataTypeDict.get('tsOffset', 0)
    if tsOffset is None:
        tsOffset = - np.min(dataTypeDict['ts'])
    dataTypeDict['ts'] = dataTypeDict['ts'] + tsOffset
    dataTypeDict['tsOffset'] = tsOffset + tsOffsetInitial

def getFirstTimestampForAChannel(channelDict):
    firstTimestamp = np.float64(np.inf)    
    for dtypeName in channelDict:
        # Probably the common format is 'ts' for all dtypes, 
        # but handle any exceptions here, example: if dtypeName == 'frame':
        try:
            firstTimestamp = min(firstTimestamp, 
                             channelDict[dtypeName]['ts'][0]) 
        except KeyError:
            # This dataType doesn't have a ts; no problem. 
            pass
    return firstTimestamp

def zeroTimestampsForAChannel(channelDict, tsOffset=None):
    if tsOffset is None:
        tsOffset = -getFirstTimestampForAChannel(channelDict)
    for dtypeName in channelDict:
        # Probably the common format is 'ts' for all dtypes, 
        # but handle any exceptions here, example: if dtypeName == 'frame':
        try:
            channelDict[dtypeName]['ts'] = channelDict[dtypeName]['ts'] + tsOffset
            channelDict[dtypeName]['tsOffset'] = tsOffset
        except KeyError:
            # This dataType doesn't have a ts; no problem. 
            pass

def rezeroTimestampsForImportedDicts(importedDicts):
    # This function assumes that timestamps have already been aligned individually. 
    if not isinstance(importedDicts, list):
        importedDicts = [importedDicts]
    # Find largest (i.e. least negative) offset
    tsOffset = np.float64(-np.inf)
    for importedDict in importedDicts:
        for channelName in importedDict['data']:
            for dataType in importedDict['data'][channelName]:
                tsOffset = max(tsOffset, importedDict['data'][channelName][dataType].get('tsOffset', -np.inf))
    # Now we have the least negative tsOffset, iterate through all, reapplying it
    for importedDict in importedDicts:
        for channelName in importedDict['data']:
            for dataType in importedDict['data'][channelName]:
                try:
                    tsOffsetCurrent = importedDict['data'][channelName][dataType].get('tsOffset', 0.0)
                    importedDict['data'][channelName][dataType]['ts'] = \
                        importedDict['data'][channelName][dataType]['ts'] + tsOffset - tsOffsetCurrent
                    importedDict['data'][channelName][dataType]['tsOffset'] = tsOffset
                except KeyError:
                    # This dataType doesn't have a ts; no problem. 
                    pass

'''
Takes ts where as int or float and returns it as float64, unwrapping where necessary.
If you pass in wrapTime that takes precedence - make sure you pass in the right type though.
If not, but you pass in tsBits (the number of timestamp bits), 
then the wrapTime is 2**wrapTime.
Otherwise, tsBits is guess by looking at the highest actual ts. 
'''
def unwrapTimestamps(ts, **kwargs):
    wrapTime = kwargs.get('wrapTime', 2**int(kwargs.get('tsBits', np.ceil(np.log2(np.max(ts)))))) # This would fail in the edge case of e.g. max ts=64, but it's highly unlikely
    # In the case that tsBits has been explicitly passed in, assume the input
    # array is uint and remove any extra bits before continuing. 
    tsBits = kwargs.get('tsBits')
    if tsBits is not None and tsBits < 32:
        ts = ts & (np.uint32(0x1 << tsBits) - 1)
    ts = ts.astype(np.float64) 
    diff = ts[1:] - ts[:-1] 
    wrapPoints = np.where(diff < 0)[0]
    for wrapPoint in wrapPoints:
        ts[wrapPoint+1:] = ts[wrapPoint+1:] + wrapTime
    return ts

# TODO: Is this obsolete?
def getFieldListForDataType(dataType):
    if dataType == 'dvs':
        return ['ts', 'x', 'y', 'pol']
    elif dataType == 'frame':
        return ['ts', 'x', 'y', 'pol']
    elif dataType == 'pose6':
        return ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    else:
        return []
    
def cropSelectedFields(dataTypeDict, fieldList, selectedBool):
    for fieldName in fieldList:
        dataTypeDict[fieldName] = dataTypeDict[fieldName][selectedBool]

'''
Accepts a container at any level of the container hierarchy and finds the highest timestamp contained
'''
def getLastTimestamp(inDict):
    lastTs = 0
    if isinstance(inDict, list):
        for inDictElement in inDict:
            lastTs = max(lastTs, getLastTimestamp(inDictElement))
    elif isinstance(inDict, dict):
        if 'ts' in inDict:
            return inDict['ts'][-1]
        else: # It's a dictionary - go through it's elements
            for keyName in inDict.keys():
                lastTs = max(lastTs, getLastTimestamp(inDict[keyName]))
    return lastTs

'''
cropDataByTimeRange was here - replaced by cropTime in split.py
'''
            
#%% LEGACY CODE - timestamps for different data types present in aedat
# There are exceptions around the timestamps for frame data to consider 
'''
def FindFirstAndLastTimeStamps(aedat):
    
    This is a sub-function of importAedat. 
    For each field in aedat['data'], it finds the first and last timestamp. 
    The min and max of these respectively are put into aedat.info
    
    
    # Clip arrays to correct size and add them to the output structure.
    # Also find first and last timeStamps
    
    if not 'data' in aedat:
        print('No data found from which to extract time stamps')
        return aedat
    
    firstTimeStamp = np.inf
    lastTimeStamp = 0
    
    if 'special' in aedat['data']:
    	if aedat['data']['special']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['special.timeStamp'][0]
    	if aedat['data']['special']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['special']['timeStamp'][-1]
    
    if 'polarity' in aedat['data']:
    	if aedat['data']['polarity']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['polarity']['timeStamp'][0]
    	if aedat['data']['polarity']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['polarity']['timeStamp'][-1]
    
    if 'frame' in aedat['data']:    
        if 'timeStampExposureStart' in aedat['data']['frame']:
            if aedat['data']['frame']['timeStampExposureStart'][0] < firstTimeStamp:
                firstTimeStamp = aedat['data']['frame']['timeStampExposureStart'][0]
            if aedat['data']['frame']['timeStampExposureEnd'][-1] > lastTimeStamp:
                lastTimeStamp = aedat['data']['frame']['timeStampExposureEnd'][-1]
        else:
            if aedat['data']['frame']['timeStampStart'][0] < firstTimeStamp:
                firstTimeStamp = aedat['data']['frame']['timeStampStart'][0]
            if aedat['data']['frame']['timeStampEnd'][-1] > lastTimeStamp:
                lastTimeStamp = aedat['data']['frame']['timeStampEnd'][-1]
    
    if 'imu6' in aedat['data']:
    	if aedat['data']['imu6']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['imu6']['timeStamp'][0]
    	if aedat['data']['imu6']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['imu6']['timeStamp'][-1]
    
    if 'sample' in aedat['data']:
    	if aedat['data']['sample']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['sample']['timeStamp'][0]
    	if aedat['data']['sample']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['sample']['timeStamp'][-1]
    
    if 'ear' in aedat['data']:
    	if aedat['data']['ear']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['ear']['timeStamp'][0]
    	if aedat['data']['ear']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['ear']['timeStamp'][-1]
    
    if 'point1D' in aedat['data']:
    	if aedat['data']['point1D']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['point1D']['timeStamp'][0]
    	if aedat['data']['point1D']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['point1D']['timeStamp'][-1]
    
    if 'point2D' in aedat['data']:
    	if aedat['data']['point2D']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['point2D']['timeStamp'][0]
    	if aedat['data']['point2D']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['point2D']['timeStamp'][-1]

    if 'point3D' in aedat['data']:
    	if aedat['data']['point3D']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['point3D']['timeStamp'][0]
    	if aedat['data']['point3D']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['point3D']['timeStamp'][-1]

    if 'point4D' in aedat['data']:
    	if aedat['data']['point4D']['timeStamp'][0] < firstTimeStamp:
    		firstTimeStamp = aedat['data']['point4D']['timeStamp'][0]
    	if aedat['data']['point4D']['timeStamp'][-1] > lastTimeStamp:
    		lastTimeStamp = aedat['data']['point4D']['timeStamp'][-1]
    
    aedat['info']['firstTimeStamp'] = firstTimeStamp
    aedat['info']['lastTimeStamp'] = lastTimeStamp

    return aedat
'''