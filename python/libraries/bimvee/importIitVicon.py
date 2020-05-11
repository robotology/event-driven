# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Suman Ghosh
        Sim Bamford
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
                    } } } }
    
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
                    } 
                'point3': {
                    'ts' : <1D array of timestamps>,
                    'point' : <2D array where each row has 3d position of a body at a time instant>,
                    'bodyId' : <1D array where each row has the bodyId of the corresponding marker>,
                    } ...
"""

import os
import re
import numpy as np

# local imports
if __package__ is None or __package__ == '':
    from timestamps import zeroTimestampsForADataType
    from split import splitByLabel
else:
    from .timestamps import zeroTimestampsForADataType
    from .split import splitByLabel

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
    pointDict = {
        'ts': poseDict['ts'][isMarker],
        'point': poseDict['point'][isMarker, :],
        'bodyId': poseDict['bodyId'][isMarker],
        }
    poseDict = {
        'ts': poseDict['ts'][~isMarker],
        'point': poseDict['point'][~isMarker, :],
        'rotation': poseDict['rotation'][~isMarker, :],
        'bodyId': poseDict['bodyId'][~isMarker],
        }
    return {
        'pose6q': poseDict,
        'point3': pointDict}
    
def importIitVicon(**kwargs):
    filePathOrName = kwargs.get('filePathOrName')
    # handle the case in which filename is not specified - iterate through files 
    # in the current directory looking for data.log
    if filePathOrName is None:
        files = [file for file in os.listdir('.') if os.path.isfile(file)]
        for filename in files:
            if filename == 'data.log':
                kwargs['filePathOrName'] = filename
                return importIitVicon(**kwargs)
        print('No suitable file found')
        return None
    pattern = re.compile('(\d+) (\d+\.\d+) \((.*)\)')
    # yarpBottleTimes = []
    outDict = {'info': {'filePathOrName': filePathOrName}, 'data': {}}
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
                poseDict['ts'].append(ts)
                poseDict['point'].append(point)
                poseDict['rotation'].append(rotation)
                poseDict['bodyId'].append(bodyId)
            line = file.readline()

    # converting lists of strings to numpy arrays of objects
    poseDict['ts'] = np.array(poseDict['ts'], dtype=np.float64)
    poseDict['point'] = np.array(poseDict['point'], dtype=np.float64)
    poseDict['rotation'] = np.array(poseDict['rotation'], dtype=np.float64)
    poseDict['bodyId'] = np.array(poseDict['bodyId'], dtype=object)
    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)):
        zeroTimestampsForADataType(poseDict)
    if kwargs.get('separateBodiesAsChannels', False):
        outDict['info']['uniqueIds'] = np.unique(poseDict['bodyId'])
        separatedBodies = splitByLabel(poseDict, 'bodyId')
        # The next line inserts the missing 'pose6q' dataType level into the hierarchy
        outDict['data'] = {bodyName: {'pose6q': bodyDict} 
                           for bodyName, bodyDict in zip(
                                   separatedBodies.keys(), 
                                   separatedBodies.values())}
    elif kwargs.get('separateMarkersFromSegments', False):
        outDict['data']['vicon'] = separateMarkersFromSegments(poseDict)
    else:            
        outDict['data']['vicon'] = {'pose6q': poseDict}
    return outDict



