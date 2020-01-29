# -*- coding: utf-8 -*-
"""
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
Contains various functions for splitting data sets:

splitByChannel
splitByPolarity
splitDvsByLabel TODO: this could be generalised to any data type ...
"""

#%%
import numpy as np

# local imports
if __package__ is None or __package__ == '':
    from timestamps import rezeroTimestampsForImportedDicts
else:
    from .timestamps import rezeroTimestampsForImportedDicts

def selectByLabel(inDict, labelName, labelValue):
    selectedEvents = inDict[labelName] == labelValue
    if not np.any(selectedEvents):
        return None
    outDict = {}
    for fieldName in inDict.keys():
        if fieldName != labelName:
            if len(inDict[fieldName]) == len(selectedEvents):
                outDict[fieldName] = inDict[fieldName][selectedEvents]
            else:
                outDict[fieldName] = inDict[fieldName]
    return outDict

def splitByLabel(inDict, labelName):
    labels = np.unique(inDict[labelName])
    outList = []
    for label in labels:
        selectedEvents = inDict[labelName] == label
        outDict = {}
        for fieldName in inDict.keys():
            try:
                assert len(inDict[fieldName]) == len(selectedEvents)
                outDict[fieldName] = inDict[fieldName][selectedEvents]
            except (TypeError, AssertionError):
                outDict[fieldName] = inDict[fieldName]
        outList.append(outDict)
    return outList

''' 
receives a dict containing (probably) dvs events
returns a dict containing two dicts, labelled 0 and 1, for the polarities found
Although redundant, the pol field is maintained within each dict for compatibility
This is similar to splitByLabel but specialised for dvs;
it is retained because True and False values make awkward dictionary keys,
so here they are replaced by strings '0' and '1'
'''
def splitByPolarity(inDict):
    outDict = {
        '0': {},
        '1': {} }
    for key in inDict:
        if type(inDict[key]) == np.ndarray:
            outDict['0'][key] = inDict[key][inDict['pol'] == 0]
            outDict['1'][key] = inDict[key][inDict['pol'] == 1]
        else:
            outDict['0'][key] = inDict[key]
            outDict['1'][key] = inDict[key]
    return outDict
    

''' 
expecting startTime, stopTime or both
If given a single dataType dict, will just cut down all arrays by masking on the ts array. 
If given a larger container, will split down all that it finds, realigning timestamps.
If the container contains an info field, then the start and stopTime params
will be added.
'''
def cropTime(inDict, **kwargs):
    if isinstance(inDict, list):
        return [cropTime(inDictInst, **kwargs) for inDictInst in inDict]
    elif 'info' in inDict:
        outDict = {'info': inDict['info'].copy(),
                   'data': {}}
        for channelName in inDict['data'].keys():
            outDict['data'][channelName] = {}
            for dataTypeName in inDict['data'][channelName].keys():
                outDict['data'][channelName][dataTypeName] = cropTime(inDict['data'][channelName][dataTypeName], **kwargs)                
        rezeroTimestampsForImportedDicts(outDict)
        return outDict
    elif 'ts' in inDict:
        ts = inDict['ts']
        if not np.any(ts): # the dataset is empty
            return inDict
        startTime = kwargs.get('startTime', kwargs.get('minTime', kwargs.get('beginTime', ts[0])))
        stopTime = kwargs.get('stopTime', kwargs.get('maxTime', kwargs.get('endTime', ts[-1])))
        if startTime == ts[0] and stopTime == ts[-1]:
            # No cropping to do - pass out the dict unmodified
            return inDict
        startIdx = np.searchsorted(ts, startTime)
        stopIdx = np.searchsorted(ts, stopTime)
        tsNew = ts[startIdx:stopIdx] - startTime
        outDict = {'ts': tsNew}
        for fieldName in inDict.keys():
            if fieldName != 'ts':
                field = inDict[fieldName]
                try:
                    outDict[fieldName] = field[startIdx:stopIdx]
                except IndexError:
                    outDict[fieldName] = field.copy() # This might fail for certain data types
        if kwargs.get('zeroTime', True):
            tsOffsetOriginal = inDict.get('tsOffset', 0)
            outDict['tsOffset'] = tsOffsetOriginal - startTime
        return outDict
    else:
        # We assume that this is a datatype which doesn't contain ts, 
        # so we pass it out unmodified
        return inDict

def cropSpace(inDict, **kwargs):
    if isinstance(inDict, list):
        return [cropSpace(inDictInst, **kwargs) for inDictInst in inDict]
    elif 'info' in inDict:
        outDict = {'info': inDict['info'].copy(),
                   'data': {}}
        for channelName in inDict['data'].keys():
            outDict['data'][channelName] = {}
            for dataTypeName in inDict['data'][channelName].keys():
                outDict['data'][channelName][dataTypeName] = cropSpace(inDict['data'][channelName][dataTypeName], **kwargs)
        # TODO: consider rezeroing space        
        return outDict
    elif 'x' in inDict and 'y' in inDict:
        x = inDict['x']
        y = inDict['y']
        if len(x) == 0: # no data to crop
            return inDict
        minX = kwargs.get('minX', np.min(x))
        maxX = kwargs.get('maxX', np.max(x))
        minY = kwargs.get('minY', np.min(y))
        maxY = kwargs.get('maxY', np.max(y))
        if (minX == np.min(x) and 
            maxX == np.max(x) and 
            minY == np.min(y) and 
            maxY == np.max(y)):
            # No cropping to do - pass out the dict unmodified
            return inDict
        selectedBool = np.logical_and(x >= minX, \
                                      np.logical_and(x <= maxX, \
                                              np.logical_and(y >= minY, y <= maxY)))
        outDict = {}
        for fieldName in inDict.keys():
            field = inDict[fieldName]
            try:
                outDict[fieldName] = field[selectedBool]
            except IndexError:
                outDict[fieldName] = field.copy() # This might fail for certain data types
        return outDict
    else:
        # We assume that this is a datatype which doesn't contain x/y
        # so we pass it out unmodified
        # TODO: frame datatype could be cropped spatially but doesn't get caught by this method
        return inDict

# synonyms
def cropSpatial(inDict, **kwargs):
    return cropSpace(inDict, **kwargs)
    
# synonyms
def cropTemporal(inDict, **kwargs):
    return cropTime(inDict, **kwargs)
    
    