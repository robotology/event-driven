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

''' 
This function receives a single importedDict from importing one file. 
It is focused on the problem of aligning data between multiple datatypes,
Within the dataType dicts there are timestamps which may or may not come from a device, like Stefi.
Let's call these data-level timestamps.
At the level of the info branch of the dict we may have system clock time 
at which a recording started.
Let's call these info-level timestamps.
This function only considers data-level timestamps.
It assumes that timestamps have already been aligned individually within datatypes.
Therefore each dataType dict which contains 'ts' should also contain 'tsOffset',
which says how much time was added to the 'ts' field.
This function aligns these so that the first event across the file is at ts = 0.
The tsOffset needed to achieve this is then added to the info branch of the dict
as tsOffsetFromData.
'''    
def rezeroTimestampsForAnImportedDict(importedDict):
    # Find largest (i.e. least negative) offset
    tsOffset = np.float64(-np.inf)
    for channelName in importedDict['data']:
        for dataType in importedDict['data'][channelName]:
            tsOffset = max(tsOffset, importedDict['data'][channelName][dataType].get('tsOffset', -np.inf))
    # Now we have the least negative tsOffset, iterate through all, reapplying it
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
    importedDict['info']['tsOffsetFromData'] = tsOffset

''' 
This function receives a list of importedDicts - each one from importing one file. 
It is focused on the problem of aligning data between multiple datatypes,
Within the dataType dicts there are timestamps which might come from a device -
let's call these data-level timestamps.
At the level of info branch of the dict we have system clock time at which 
a recording started - Let's call these info-level timestamps.
It first calls the above function rezeroTimestampsForAnImportedDict for each file, 
so that timestamps are aligned individually within the dict for each file. 
The timestamp which was used to achieve this will be in ['info']['tsOffsetFromData'].
There may additionally be ['info']['tsOffsetFromInfo'] - the info level timestamps.
If info/level timestamops are available for all files, these are used to rezero
all the data across multiple files.
If any are absent, then they are ignored, and alignment across multiple files
is based on data-level timestamps.
'''      
def rezeroTimestampsForImportedDicts(importedDicts):
    if not isinstance(importedDicts, list):
        importedDicts = [importedDicts]
    # Find largest (i.e. least negative) offset
    tsOffsetFromData = np.float64(-np.inf)
    # Confusingly, info-level timestamps run in the other direction! TODO: could fix this
    tsOffsetFromInfo = np.float64(np.inf)
    allHaveInfoTsOffset = True
    for importedDict in importedDicts:
        rezeroTimestampsForAnImportedDict(importedDict)
        tsOffsetFromData = max(tsOffsetFromData, importedDict['info']['tsOffsetFromData'])
        if importedDict['info'].get('tsOffsetFromInfo', 0) != 0: # 0 is a placeholder used by the exporter - if it's zero, we assume it is not a real value that we care about.
            tsOffsetFromInfo = min(tsOffsetFromInfo, importedDict['info']['tsOffsetFromInfo'])
        else:
            allHaveInfoTsOffset = False
    if len(importedDicts) == 1:
        return
    # Now we have the extreme tsOffsets, iterate through all, reapplying 
    for importedDict in importedDicts:
        if allHaveInfoTsOffset and len(importedDicts) > 1:
            if tsOffsetFromInfo == importedDict['info']['tsOffsetFromInfo']:
                # The reason for this if clause is to catch a floating point 
                # precision error due to mixing very big and very small timestamps. 
                tsOffset = tsOffsetFromData                
            else:
                tsOffset = importedDict['info']['tsOffsetFromData'] - tsOffsetFromInfo + importedDict['info']['tsOffsetFromInfo']
            
        else:
            tsOffset = tsOffsetFromData
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
using container as a general term for datatype dicts, channels, fileDicts or any hierarchical list of them...
recurse through the structure, modifying timestamps and tsOffsets by a fixed amount
'''
def offsetTimestampsForAContainer(container, offset):
    if isinstance(container, list):
        for elem in container:
            offsetTimestampsForAContainer(elem, offset)
        return
    if isinstance(container, dict):
        if 'ts' in container:
            # It's a datatype dict
            container['ts'] = container['ts'] + offset
            container['tsOffset'] = container['tsOffset'] + offset
        else:
            for field in container.values():
                offsetTimestampsForAContainer(field, offset)
    else:
        # We have descended too far
        return

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

'''
Sort a dict containing ts according to ts, applying the new sort order to all
the fields where axis 0 
'''
def sortDataTypeDictByTime(inDict):
    ts = inDict['ts']
    ids = np.argsort(ts) 
    numEvents = ts.shape[0]
    outDict = {}
    for fieldName in inDict.keys():
        try:
            assert len(inDict[fieldName]) == numEvents
            outDict[fieldName] = inDict[fieldName][ids]
        except (AssertionError, TypeError):
            outDict[fieldName] = inDict[fieldName]
    return outDict
            
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