# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Simeon Bamford, Aiko Dinale
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
This script contains a set of examples of how to import and export data 
from and to (large) files, one batch at a time.
In each case, change the file paths as required to point to your own example data.
"""

#%% Preliminaries

import os, sys # A system-specific prefix, for working between linux and windows
prefix = 'C:/' if os.name == 'nt' else '/home/adinale/'
# A path to the location of bimvee library, if it is not installed
sys.path.append(os.path.join(prefix, 'src/event-driven/python/libraries'))

#%%

from bimvee.importIitYarp import importIitYarp
from bimvee.exportIitYarp import exportIitYarp
from bimvee.info import info
from bimvee.timestamps import offsetTimestampsForAContainer

dataset_root = "Documents/Event-Driven_Dataset/datasetIMU/roundTable/"
dataset_name = "roundTable_6DOF_004"
filePathOrName = os.path.join(prefix, dataset_root, dataset_name)

#%%
# Read a first batch

container1 = importIitYarp(filePathOrName=filePathOrName,
                          tsBits=30,
                          convertSamplesToImu=False,
                          importFromByte=0,
                          importMaxBytes=1000000)
info(container1)

importedToByte = container1['info']['importedToByte']
tsOffsetContainer1 = container1['info']['tsOffsetFromData']
#tsOffsetFromInfo = container1['info']['tsOffsetFromInfo']

# Export first batch

exportIitYarp(container1,
            exportFilePath= filePathOrName + "_batch01",
            pathForPlayback= filePathOrName + "_batch01",
            dataTypes = ['sample', 'dvs'],
            protectedWrite = False)

#%%

# Read a second batch

container2 = importIitYarp(filePathOrName=filePathOrName,
                          tsBits = 30,
                          convertSamplesToImu=False,
                          importFromByte=importedToByte+1,
                          importMaxBytes=1000000)

exportIitYarp(container2,
            exportFilePath= filePathOrName + "_batch02",
            pathForPlayback= filePathOrName + "_batch02",
            dataTypes = ['sample', 'dvs'],
            protectedWrite = False)

#%%

# Offset timestamps in the second batch

tsOffsetContainer2 = container2['info']['tsOffsetFromData']
tsOffset = container1['info']['tsOffsetFromData']
offsetToApplyToContainer2 = tsOffsetContainer1 - tsOffsetContainer2
offsetTimestampsForAContainer(container2, offsetToApplyToContainer2)
container2['info']['tsOffsetFromData'] += offsetToApplyToContainer2
info(container2)

# Export first and second batches

exportIitYarp(container1,
            exportFilePath= filePathOrName + "_batch12",
            pathForPlayback= filePathOrName + "_batch12",
            dataTypes = ['sample', 'dvs'],
            protectedWrite = False)

exportIitYarp(container2,
            exportFilePath= filePathOrName + "_batch12",
            pathForPlayback= filePathOrName + "_batch12",
            dataTypes = ['sample', 'dvs'],
            protectedWrite = False,
            writeMode = 'a')
