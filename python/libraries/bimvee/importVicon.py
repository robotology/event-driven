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
                    'pose' : <2D array where each row has 7d pose of a body at a time instant>} } } }
Otherwise:
    outDict = {
        'info': {
            'filePathOrName': <filepath>
            }, 
        'data': {
            'vicon': {
                'pose6q': {
                    'ts' : <1D array of timestamps>,
                    'pose' : <2D array where each row has 7d pose of a body at a time instant>,
                    'bodyId' : <1D array where each row has the bodyId of the corresponding marker>,
                    'uniqueIds' : <1D array of strings, one for each unique marker>} } } }
    
A bodyID is the name assigned by vicon to a marker (labeled / unlableled) or rigid body
The 7d pose is in the form [x, y, z, r_x, r_y, r_z, r_w] with the orientation as a quaternion
The datatype is called 'pose6q', referring to the 6dof with rotation in quaternion form.

Additionally, if separateBodiesAsChannels is not present or false, 
and the separateMarkersAndSegments parameter is present and True,
then the data in the vicon channel is broken into two datatypes:
        ...
            'vicon': {
                'pose6q': {
                    'ts' : <1D array of timestamps>,
                    'pose' : <2D array where each row has 7d pose of a body at a time instant>,
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
from timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts

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
    isMarker = np.apply_along_axis(lambda x : 'Marker' in x[0], 1, poseDict['bodyId'][..., np.newaxis])
    uniqueIdIsMarker = np.apply_along_axis(lambda x : 'Marker' in x[0], 1, poseDict['uniqueIds'][..., np.newaxis])
    pointDict = {
        'ts': poseDict['ts'][isMarker],
        'point': poseDict['pose'][isMarker, :3],
        'bodyId': poseDict['bodyId'][isMarker],
        'uniqueIds': poseDict['uniqueIds'][uniqueIdIsMarker],
        }
    poseDict = {
        'ts': poseDict['ts'][~isMarker],
        'pose': poseDict['pose'][~isMarker],
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
        poseDict = {'ts': [], 'pose': [], 'bodyId': []}
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
                pose = elements[3:]
                if separateBodiesAsChannels:
                    try:
                        poseDict = outDict['data'][bodyId]['pose6q']
                    except KeyError:
                        print('KeyError exception.. Creating new key', bodyId)
                        uniqueIds.append(bodyId)
                        outDict['data'][bodyId] = {'pose6q': {'ts': [], 'pose': []}}
                        poseDict = outDict['data'][bodyId]['pose6q']
                poseDict['ts'].append(ts)
                poseDict['pose'].append(pose)
                if not separateBodiesAsChannels:
                    poseDict['bodyId'].append(bodyId)
            line = file.readline()

    # converting lists of strings to numpy arrays of float64
    if separateBodiesAsChannels:
        for id in uniqueIds:
            outDict['data'][id]['pose6q']['ts'] = np.array(outDict['data'][id]['pose6q']['ts'], dtype=np.float64)
            outDict['data'][id]['pose6q']['pose'] = np.array(outDict['data'][id]['pose6q']['pose'], dtype=np.float64)
            if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
                zeroTimestampsForAChannel(outDict['data'][id])
        if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
            rezeroTimestampsForImportedDicts(outDict)
        outDict['info']['uniqueIds'] = uniqueIds
    else:
        poseDict['ts'] = np.array(poseDict['ts'], dtype=np.float64)
        poseDict['pose'] = np.array(poseDict['pose'], dtype=np.float64)
        poseDict['bodyId'] = np.array(poseDict['bodyId'])
        poseDict['uniqueIds'] = np.unique(poseDict['bodyId'])
        if kwargs.get('separateMarkersFromSegments', False):
            outDict['data']['vicon'] = separateMarkersFromSegments(poseDict)
        else:            
            outDict['data']['vicon'] = {'pose6q': poseDict}
            
        if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
            zeroTimestampsForAChannel(outDict['data']['vicon']) # TODO: Zeroing in the separated channel case
    return outDict



