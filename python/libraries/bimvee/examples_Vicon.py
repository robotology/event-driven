# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Suman Ghosh
         Simeon Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
This script contains a set of examples of how to use the functions of the bimvee library.
In each case, change the file paths as required to point to your own example data.
"""


#%% Preliminaries

import os, sys # A system-specific prefix, for working between linux and windows
prefix = 'C:/' if os.name == 'nt' else '/home/sbamford/'

sys.path.append(os.path.join(prefix, 'repos/event-driven-python-dev/python/libraries/bimvee')) # A path to this library

#%% Import with each body as separate channel

from importVicon import importVicon

filePathOrName = os.path.join(prefix, '/data/2019_12_12_vicon/Trial2WithVicon/Vicon/data.log')
kwargs = {'filePathOrName': filePathOrName,
          'separateBodiesAsChannels': True}
viconDataDict = importVicon(**kwargs)
if 'bodyids' in viconDataDict['info']:
    print('The parsed body IDs are: ', viconDataDict['info']['bodyids'])

#%% Import with all bodies in the same channel

from importVicon import importVicon

viconDataDict = importVicon(filePathOrName)
if 'bodyids' in viconDataDict['info']:
    print('The parsed body IDs are: ', viconDataDict['info']['bodyids'])

#%% Import with all bodies in the same channel, with markers separated from segments

from importVicon import importVicon

viconDataDict = importVicon(filePathOrName, separateMarkersFromSegments=True)
if 'bodyids' in viconDataDict['info']:
    print('The parsed body IDs are: ', viconDataDict['info']['bodyids'])

#%%

from plotViconTrajectory import plot_trajectories
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

uniqueIds = viconDataDict['info']['uniqueIds'] 

# StEFI body trajectory
fig = plt.figure(1)
ax = plt.axes(projection='3d')
include = ['StEFI']
exclude = ['Marker']
plot_trajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

# StEFI labeled markers trajectory
fig = plt.figure(2)
ax = plt.axes(projection='3d')
include = ['StEFI', 'Marker']
exclude = []
plot_trajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

# Unlabeled markers trajectory
fig = plt.figure(3)
ax = plt.axes(projection='3d')
include = ['UnlMarker']
exclude = []
plot_trajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

plt.show()