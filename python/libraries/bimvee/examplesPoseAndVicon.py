# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Simeon Bamford
        Suman Ghosh
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
This script contains a set of examples of how to use the 'pose6q' data type
of the bimvee library.
In each case, change the file paths as required to point to your own example data.
"""

#%% Preliminaries

import os, sys # A system-specific prefix, for working between linux and windows
prefix = 'C:/' if os.name == 'nt' else '/home/sbamford/'
# A path to the location of this library, if not installed as a package
sys.path.append(os.path.join(prefix, 'repos/event-driven-poseExamples/python/libraries')) 
sys.path.append(os.path.join(prefix, 'repos/event-driven/python'))

#%% Import with all bodies in the same channel

from bimvee.importIitVicon import importIitVicon

viconDataDict = importIitVicon(filePathOrName=filePathOrName)
if 'uniqueIds' in viconDataDict['data']['vicon']['pose6q']:
    print('The parsed body IDs are: ', viconDataDict['data']['vicon']['pose6q']['uniqueIds'])

#%% Import with all bodies in the same channel, with markers separated from segments

from bimvee.importVicon import importVicon

filePathOrName = os.path.join(prefix, '/data/2019_12_12_vicon/Trial2WithVicon/Vicon/data.log')
viconDataDict = importIitVicon(filePathOrName=filePathOrName, separateMarkersFromSegments=True)
if 'uniqueIds' in viconDataDict['data']['vicon']['pose6q']:
    print('The parsed body IDs are: ', viconDataDict['data']['vicon']['pose6q']['uniqueIds'])
if 'uniqueIds' in viconDataDict['data']['vicon']['point3']:
    print('The parsed marker IDs are: ', viconDataDict['data']['vicon']['point3']['uniqueIds'])

#%% Import with each body as separate channel

from bimvee.importIitVicon import importIitVicon

filePathOrName = os.path.join(prefix, '/data/2019_12_12_vicon/Trial2WithVicon/Vicon/data.log')
kwargs = {'filePathOrName': filePathOrName,
          'separateBodiesAsChannels': True}
viconDataDict = importIitVicon(**kwargs)
if 'uniqueIds' in viconDataDict['info']:
    print('The parsed body IDs are: ', viconDataDict['info']['uniqueIds'])

#%%

from bimvee.plotPose import plotTrajectories
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

uniqueIds = viconDataDict['info']['uniqueIds'] 

# StEFI body trajectory
fig = plt.figure(1)
ax = plt.axes(projection='3d')
include = ['StEFI']
exclude = ['Marker']
plotTrajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

# StEFI labeled markers trajectory
fig = plt.figure(2)
ax = plt.axes(projection='3d')
include = ['StEFI', 'Marker']
exclude = []
plotTrajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

# Unlabeled markers trajectory
fig = plt.figure(3)
ax = plt.axes(projection='3d')
include = ['UnlMarker']
exclude = []
plotTrajectories(viconDataDict, uniqueIds, include, exclude, ax=ax)

plt.show()


#%% Import poses from Vicon and select only one body for further analysis

from bimvee.importIitVicon import importIitVicon
from bimvee.split import selectByLabel

filePathOrName = os.path.join(prefix, '/data/2019_12_12_vicon/Trial2WithVicon/Vicon/data.log')
kwargs = {'filePathOrName': filePathOrName,
          'separateBodiesAsChannels': False}
viconDataDict = importIitVicon(**kwargs)
posesForSelectedBody = selectByLabel(viconDataDict['data']['vicon']['pose6q'], 'bodyId', 'Subj_StEFI::Seg_body')
del posesForSelectedBody['uniqueIds']

dataDict = {'pose6q': posesForSelectedBody}

#%% Remove erroneous pose samples

# We observe null poses in our data, elimnate these. In practice, these are set 
# to point=0,0,0 rotation=0,0.5,0,0  so one way to tell that they're
# erroneous is to test for magnitude != unity

import numpy as np

rotation = dataDict['pose6q']['rotation']
magnitude = np.sum(rotation ** 2, axis = 1) 
toKeep = magnitude == 1

dataDict['pose6q']['toKeep'] = toKeep
dataDict['pose6q'] = selectByLabel(dataDict['pose6q'], 'toKeep', True)

#%% Interpolate poses - make sure there is at least one sample every 10 ms

from bimvee.geometry import pose6qInterp
                                         
dataDict['pose6q'] = pose6qInterp(dataDict['pose6q'], maxPeriod=0.005)


#%% Construct a pose sequence by hand

''' 
This example section makes a series of movements, with range of about 1m,
over a period of 4 seconds, which between them, cover all basic degrees of 
freedom in translation and rotation; it produces a sample every 10 ms. 
'''

import numpy as np

ts = np.arange(0, 4.01, 0.01, dtype=np.float64)
point = np.zeros((401, 3), dtype=np.float64)
rotation = np.zeros((401, 4), dtype=np.float64)
rotation[:, 0] = 1 # Neutral quaternion pose

# First: yaw-through-pitch anticlockwise

rotation[:100, 1] = np.sin(ts[:100] * np.pi * 2) / 4 
rotation[:100, 2] = (np.cos(ts[:100] * np.pi * 2) - 1) / 4 
rotation[:100, 0] = np.sqrt(1 - (rotation[:100, 1] ** 2 + rotation[:100, 2] ** 2))

# Second: translation anticlockwise: 

point[100:200, 0] = np.sin(ts[100:200] * np.pi * 2) / 2 
point[100:200, 1] = (np.cos(ts[100:200] * np.pi * 2) - 1) / 2 
    
# Third: Out-in along z.

point[200:250, 2] = np.arange(0, -1, -0.02)
point[250:300, 2] = np.arange(-1, 0, 0.02)

# Fourth: roll anticlockwise around z:

rotation[300:350, 3] = -np.sqrt(np.arange(0, 1, 0.02))
rotation[300:350, 0] = np.sqrt(np.arange(1, 0, -0.02))
rotation[350:400, 3] = np.sqrt(np.arange(1, 0, -0.02))
rotation[350:400, 0] = np.sqrt(np.arange(0, 1, 0.02))

# Optionally, add an arbitrary translational shift to the whole sequence

point[:, 2] = point[:, 2] - 3
point[:, 1] = point[:, 1] + 3

# Construct the result as a bimvee-style container
dataDict = {
    'pose6q': {
        'ts': ts,
        'point': point,  
        'rotation': rotation
        }}  

#%% 
'''
The above poses are from an initial viewpoint of the camera.
We now convert to the coordinate frame of the simluated world:
The camera is at x=1, y=0, z=1.5, so it's in front of the wall where the square is,
And it's oriented with:
    camera z along world -x, 
    camera x along y
    camera y along -z
'''

from bimvee.geometry import combineTwoQuaternions
from pyquaternion import Quaternion as Q

# Use pyquaternion library to give us a quaternion starting with a rotation matrix
cameraToWorldRotMat = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=np.float64)
cameraToWorldQ = Q(matrix = cameraToWorldRotMat).q

rotationNew = np.zeros_like(rotation)

for idx in range(401):
    rotationNew[idx, :] = combineTwoQuaternions(rotation[idx, :], cameraToWorldQ)

# Same business for the points
    
pointNew = np.zeros_like(point)
pointNew[:, 0] = -point[:, 2] + 1
pointNew[:, 1] = point[:, 0]
pointNew[:, 2] = -point[:, 1] + 1.5

dataDict['pose6q']['point'] = pointNew  
dataDict['pose6q']['rotation'] = rotationNew  

#%% Try to visualise that
        
import visualizer.ntupleviz
import threading

# Create the dualViz app and start it in a thread
visualizerApp = visualizer.ntupleviz.Ntupleviz()
thread = threading.Thread(target=visualizerApp.run)
thread.daemon = True
thread.start()

#%% Visualise

visualizerApp.root.data_controller.data_dict = dataDict

#%% Export a posedict as csv for eSim simulator

from bimvee.exportPoseRpgEsimCsv import exportPoseRpgEsimCsv

filePathAndName = os.path.join(prefix, 'data/poses.csv')
exportPoseRpgEsimCsv(dataDict['pose6q'], filePathAndName=filePathAndName)

#%% Import the resulting simulation
        
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/2020_02_13 simulation square/simple_square_far.bag')

template = {
    'davis': {
        'dvs': '/cam0/events',
        'frame': '/cam0/image_raw',
        'imu': '/imu'
        },
    'extra': {
        'pose6q': '/cam0/pose',
        'frame':  '/cam0/depthmap',
    }}
#        'cam': '/cam0/camera_info',

imported = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

f = {'frame': imported['data']['extra']['frame']}
