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

import numpy as np
import pprint
pp = pprint.PrettyPrinter(indent=12)

def fieldMinMax(dataTypeDict, fieldName):
    if fieldName in dataTypeDict:
        print('            ', 
              np.min(dataTypeDict[fieldName]), 
              ' >= ', fieldName, 
              ' >= ', np.max(dataTypeDict[fieldName]))

def infoForImportedDicts(importedDicts, **kwargs):
    if not isinstance(importedDicts, list):
        importedDicts = [importedDicts]
    for importedDict in importedDicts:
        print(importedDict['info'])
        for channelName in importedDict['data']:
            print('    Channel: ' + channelName)
            for dataType in importedDict['data'][channelName]:
                print('        DataType: ' + dataType)
                dataTypeDict = importedDict['data'][channelName][dataType]
                if 'ts' in dataTypeDict:
                    print('            Num events: ', len(dataTypeDict['ts']))
                    fieldMinMax(dataTypeDict, 'ts')
                else:
                    pp.pprint(dataTypeDict)
                if 'tsOffset' in dataTypeDict:
                    print('            Ts offset: ', dataTypeDict['tsOffset'])
                for dataType in ['pol', 'x', 'y']:
                    fieldMinMax(dataTypeDict, 'pol')

def infoTsForImportedDicts(importedDicts, **kwargs):
    if not isinstance(importedDicts, list):
        importedDicts = [importedDicts]
    for importedDict in importedDicts:
        print(importedDict['info'])
        for channelName in importedDict['data']:
            print('    Channel: ' + channelName)
            for dataType in importedDict['data'][channelName]:
                print('        DataType: ' + dataType)
                dataTypeDict = importedDict['data'][channelName][dataType]
                if 'ts' in dataTypeDict:
                    fieldMinMax(dataTypeDict, 'ts')
                if 'tsOffset' in dataTypeDict:
                    print('            Ts offset: ', dataTypeDict['tsOffset'])

def info(importedDicts, **kwargs):
    infoForImportedDicts(importedDicts, **kwargs)