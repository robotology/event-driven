# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Suman Ghosh
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Input:
vicon data log recorded through yarp

There are two output formats. If 'parameter' separateBodiesAsChannels is present 
and True then the format is:
    outDict = {
        'info': {
            'filePathOrName': <filepath>
            'uniqueIds': np.array of strings, one for each unique bodyId 
            }, 
        'data': { 
            <bodyID>: {
                'pose6q': {
                    'ts' : <1D array of timestamps>,
                    'point' : <2D array where each row has 3d position of a body at a time instant>,
                    'rotation' : <2D array where each row has rotation of a body at a time instant expressed as a quaternion (4d)>} } } }
Otherwise:
    outDict = {
        'info': {
            'filePathOrName': <filepath>
            }, 
        'data': {
            'vicon': {
                'pose6q': {
                    'ts' : <1D array of timestamps>,
                    'point' : <2D array where each row has 3d position of a body at a time instant>,
                    'rotation' : <2D array where each row has rotation of a body at a time instant expressed as a quaternion (4d)>,
                    'bodyId' : <1D array where each row has the bodyId of the corresponding marker>,
                    'uniqueIds' : <1D array of strings, one for each unique marker>} } } }
    
A bodyID is the name assigned by vicon to a marker (labeled / unlableled) or rigid body
The pose consists of a point in the form [x, y, z]
and a rotation as a quaternion [r_w, r_x, r_y, r_z] (Caution with alternative orderings here)
The datatype is called 'pose6q', referring to the 6dof with rotation in quaternion form.

Additionally, if separateBodiesAsChannels is not present or false, 
and the separateMarkersAndSegments parameter is present and True,
then the data in the vicon channel is broken into two datatypes:
        ...
            'vicon': {
                'pose6q': {
                    'ts' : <1D array of timestamps>,
                    'point' : <2D array where each row has 3d position of a body at a time instant>,
                    'rotation' : <2D array where each row has the rotation of a body at a time instant expressed as a quaternion (4d)>,
                    'bodyId' : <1D array where each row has the bodyId of the corresponding marker>,
                    'uniqueIds' : <1D array of strings, one for each unique marker>} 
                'point3': {
                    'ts' : <1D array of timestamps>,
                    'point' : <2D array where each row has 3d position of a body at a time instant>,
                    'bodyId' : <1D array where each row has the bodyId of the corresponding marker>,
                    'uniqueIds' : <1D array of strings, one for each unique marker>} } ...
"""

import re
import numpy as np

# local imports
if __package__ is None or __package__ == '':
    from timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts
else:
    from .timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts

def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value

# accepts a pose6q datatype dict; returns a channel dict containing pose6q and point3 datatypes
def separateMarkersFromSegments(poseDict):
    isMarker = np.apply_along_axis(lambda x : 'Marker' in str(x[0]), 1, poseDict['bodyId'][..., np.newaxis])
    uniqueIdIsMarker = np.apply_along_axis(lambda x : 'Marker' in str(x[0]), 1, poseDict['uniqueIds'][..., np.newaxis])
    pointDict = {
        'ts': poseDict['ts'][isMarker],
        'point': poseDict['point'][isMarker, :],
        'bodyId': poseDict['bodyId'][isMarker],
        'uniqueIds': poseDict['uniqueIds'][uniqueIdIsMarker],
        }
    poseDict = {
        'ts': poseDict['ts'][~isMarker],
        'point': poseDict['point'][~isMarker, :],
        'rotation': poseDict['rotation'][~isMarker, :],
        'bodyId': poseDict['bodyId'][~isMarker],
        'uniqueIds': poseDict['uniqueIds'][~uniqueIdIsMarker],
        }
    return {
        'pose6q': poseDict,
        'point3': pointDict}

def importVicon(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    pattern = re.compile('(\d+) (\d+\.\d+) \((.*)\)')
    # yarpBottleTimes = []
    outDict = {'info': {'filePathOrName': filePathOrName}, 'data': {}}
    separateBodiesAsChannels = kwargs.get('separateBodiesAsChannels', False)
    if separateBodiesAsChannels:
        uniqueIds = []
    else: 
        poseDict = {'ts': [], 'point': [], 'rotation': [], 'bodyId': []}
    with open(filePathOrName, 'r') as file:
        print('Found file to read')
        line = file.readline()
        while line:
            found = pattern.search(line.strip())
            # yarpBottleTimes.append(float(found.group(2)))
            viconData = found.group(3)
            bodies = viconData.split(') (')
            for body in bodies:
                elements = body.split(" ")
                bodyId = elements[1].strip('\"')
                ts = elements[2]
                point = elements[3:6]
                # Note: quaternion order is [w,x,y,z] - this is defined by yarp 
                # IFrameTransform component, so ignore vicon documentation
                rotation = elements[6:] 
                if separateBodiesAsChannels:
                    try:
                        poseDict = outDict['data'][bodyId]['pose6q']
                    except KeyError:
                        print('KeyError exception.. Creating new key', bodyId)
                        uniqueIds.append(bodyId)
                        outDict['data'][bodyId] = {'pose6q': {'ts': [], 'point': [], 'rotation': []}}
                        poseDict = outDict['data'][bodyId]['pose6q']
                poseDict['ts'].append(ts)
                poseDict['point'].append(point)
                poseDict['rotation'].append(rotation)
                if not separateBodiesAsChannels:
                    poseDict['bodyId'].append(bodyId.encode('utf-8'))
            line = file.readline()

    # converting lists of strings to numpy arrays of objects
    if separateBodiesAsChannels:
        for id in uniqueIds:
            outDict['data'][id]['pose6q']['ts'] = np.array(outDict['data'][id]['pose6q']['ts'], dtype=np.float64)
            outDict['data'][id]['pose6q']['point'] = np.array(outDict['data'][id]['pose6q']['point'], dtype=np.float64)
            outDict['data'][id]['pose6q']['rotation'] = np.array(outDict['data'][id]['pose6q']['rotation'], dtype=np.float64)
            if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
                zeroTimestampsForAChannel(outDict['data'][id])
        if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
            rezeroTimestampsForImportedDicts(outDict)
        outDict['info']['uniqueIds'] = uniqueIds
    else:
        poseDict['ts'] = np.array(poseDict['ts'], dtype=np.float64)
        poseDict['point'] = np.array(poseDict['point'], dtype=np.float64)
        poseDict['rotation'] = np.array(poseDict['rotation'], dtype=np.float64)
        poseDict['bodyId'] = np.array(poseDict['bodyId'])
        poseDict['uniqueIds'] = np.unique(poseDict['bodyId'])
        if kwargs.get('separateMarkersFromSegments', False):
            outDict['data']['vicon'] = separateMarkersFromSegments(poseDict)
        else:            
            outDict['data']['vicon'] = {'pose6q': poseDict}
            
        if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
            zeroTimestampsForAChannel(outDict['data']['vicon']) # TODO: Zeroing in the separated channel case
    return outDict



