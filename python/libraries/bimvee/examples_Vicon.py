# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Suman Ghosh
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
In each case, change the file paths as required to point toyour own example data.
"""


#%% Importing an visualising data from Vicon

from importVicon import importVicon
from plotViconTrajectory import plot_trajectories
import matplotlib.pyplot as plt

import os, sys # A system-specific prefix, for working between linux and windows
if os.name == 'nt':
    prefix = 'C:/'
else:
    prefix = '/home/sbamford/'    
    
sys.path.append(os.path.join(prefix, 'repos/Sim/bimvee')) # A path to this library

filePathOrName = os.path.join(prefix, '/data/2019_12_12_vicon/Vicon/boardStEFI001/data.log')

viconDataDict, bodyids = importVicon(filePathOrName)
print('The parsed body IDs are: ', bodyids)

# StEFI body trajectory
fig = plt.figure(1)
ax = plt.axes(projection='3d')
include = ['StEFI']
exclude = ['Marker']
ax = plot_trajectories(ax, viconDataDict, bodyids, include, exclude)

# StEFI labeled markers trajectory
fig = plt.figure(2)
ax = plt.axes(projection='3d')
include = ['StEFI', 'Marker']
exclude = []
ax = plot_trajectories(ax, viconDataDict, bodyids, include, exclude)

# Unlabeled markers trajectory
fig = plt.figure(3)
ax = plt.axes(projection='3d')
include = ['UnlMarker']
exclude = []
ax = plot_trajectories(ax, viconDataDict, bodyids, include, exclude)

plt.show()