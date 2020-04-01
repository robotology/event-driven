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
Contains general functions for text summaries of the contents of dicts which result 
from imports using importAe function
"""

from math import log10, floor
import numpy as np
import pprint
pp = pprint.PrettyPrinter(indent=12)

# Round to 3 s.f.
def sf3(x):
    if x and isinstance(x, (int, float, complex)) and not isinstance(x, bool):
        return round(x, -int(floor(log10(abs(x)))) + 2)
    else:
        return x

def fieldMinMax(dataTypeDict, fieldName):
    if fieldName in dataTypeDict:
        field = dataTypeDict[fieldName]
        if type(field) == np.ndarray:
            if field.shape[0] > 1:
                # Handle 2D arrays, e.g. an array containing x, y, z in columns
                if len(field.shape) > 1:
                    for dim1Idx in range(field.shape[1]):
                        print('            ', 
                              sf3(np.min(field[:, dim1Idx])), 
                              ' >= ', fieldName, ' - col ', dim1Idx,  
                              ' >= ', sf3(np.max(field[:, dim1Idx])))
                else:
                    print('            ', 
                          sf3(np.min(field)), 
                          ' >= ', fieldName,
                          ' >= ', sf3(np.max(field)))

def info(containers, **kwargs):
    if not isinstance(containers, list):
        containers = [containers]
    for container in containers:
        print(container['info'])
        for channelName in container['data']:
            print('    Channel: ' + channelName)
            for dataType in container['data'][channelName]:
                print('        DataType: ' + dataType)
                dataTypeDict = container['data'][channelName][dataType]
                if 'ts' in dataTypeDict:
                    print('            Num events: ', len(dataTypeDict['ts']))
                    fieldMinMax(dataTypeDict, 'ts')
                    if 'tsOffset' in dataTypeDict:
                        print('            Ts offset: ', dataTypeDict['tsOffset'])
                    for fieldName in dataTypeDict.keys():
                        if fieldName not in ['ts', 'tsOffset']:
                            fieldMinMax(dataTypeDict, fieldName)
                else:
                    pp.pprint(dataTypeDict)
                print()
            print()
def infoTs(containers, **kwargs):
    if not isinstance(containers, list):
        containers = [containers]
    for container in containers:
        print(container['info'])
        for channelName in container['data']:
            print('    Channel: ' + channelName)
            for dataType in container['data'][channelName]:
                print('        DataType: ' + dataType)
                dataTypeDict = container['data'][channelName][dataType]
                if 'ts' in dataTypeDict:
                    fieldMinMax(dataTypeDict, 'ts')
                if 'tsOffset' in dataTypeDict:
                    print('            Ts offset: ', dataTypeDict['tsOffset'])

#%% Legacy function names

def infoForImportedDicts(container, **kwargs):
    info(container, **kwargs)

def infoTsForImportedDicts(container, **kwargs):
    infoTs(container, **kwargs)