# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
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

Using hickle to add hierarchical lists and dicts to hdf5 automatically
https://github.com/telegraphic/hickle
In fact, thess are just thin wrappers around hickle.dump/load, 
to offer a similar export function to other export calls.
pack/unpackWorkspaceVars are facilities that allows extra random data WIP 
to get bundled together. All files are included in both the import and export script for convenience. 
"""

#%%

import hickle
import os

def exportHdf5(data, exportFilePathAndName='./temp.hdf5'):
    if exportFilePathAndName[-5:] != '.hdf5':
        exportFilePathAndName = exportFilePathAndName + '.hdf5'
    print('exportHdf5 called, targeting file path and name' + exportFilePathAndName)
    absPath = os.path.dirname(os.path.abspath(exportFilePathAndName))
    if not os.path.exists(absPath):
        os.mkdir(absPath)
    hickle.dump(data, exportFilePathAndName)
    
def importHdf5(filePathOrName='./temp.hdf5'):
    #TODO: Handle path with no filename
    if filePathOrName[-5:] != '.hdf5':
        filePathOrName = filePathOrName + '.hdf5'
    print('importHdf5 called, targeting file path and name' + filePathOrName)
    return hickle.load(filePathOrName)

#%% WIP - the following functions aren't working yet

def pack(inDict, names):
    packWorkspaceVars(inDict, names)
    
def unpack(inDict):
    unpackWorkspaceVars(inDict)
    
def packWorkspaceVars(inDict, names):
    if isinstance(inDict, list):
        return packWorkspaceVars(inDict[0], names)
    if 'temp' not in inDict['info']:
        inDict['info']['temp'] = {}
    if not isinstance(names, list):
        names = [names]
    for name in names:
        exec('global ' + name)
        exec("inDict['info']['temp'][name] = globals()[" + name + "]")

def unpackWorkspaceVars(inDict):
    if 'info' in inDict and 'temp' in inDict['info']:
        for key in list(inDict['info']['temp'].keys()):
            exec('global ' + key)
            globals()[key] = inDict['info']['temp'].pop(key)
