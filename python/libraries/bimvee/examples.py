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
This script contains a set of examples of how to use the functions of the bimvee library.
In each case, change the file paths as required to point toyour own example data.
"""

#%% Preliminaries - set your paths as necessary

import os, sys # A system-specific prefix, for working between linux and windows
prefix = 'C:/' if os.name == 'nt' else '/home/sbamford/'
    
sys.path.append(os.path.join(prefix, 'repos/event-driven-python-dev/python/libraries/bimvee')) # A path to this library
#sys.path.insert(0, os.path.join(prefix, 'repos/event-driven-python-dev/python/libraries/bimvee')) # A path to this library

#%% IMPORT FUNCTIONS

#%% Import from yarp

from importIitYarp import importIitYarp

filePathOrName = os.path.join(prefix, "data/2019_11_11_AikoImu/tripod_pitch")
# If the number of bits dedicated to the timestamp have been limited
# prior to dumping, then match this with the 'tsBits' parameter
imported = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

#%% Inspection of a rosbag
'''
To just inspect the connections of a rosbag file , pass in an empty template.
This doesn't import any data but prints out which connections are present 
in the file. You can then construct a template to import the file as desired.
'''

from importRpgDvsRos import importRpgDvsRos
filePathOrName = os.path.join(prefix, 'data/rpg/shapes_rotation.bag')
inspected = importRpgDvsRos(filePathOrName=filePathOrName)

#%% Import Rpg Event-Camera Dataset

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/shapes_rotation.bag')

template = {
    'ch0': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        'pose6q': '/optitrack/davis',
        'cam': '/dvs/camera_info',
        'imu': '/dvs/imu'
        }
    }

imported = importRpgDvsRos(filePathOrName=filePathOrName, template=template)

#%% import Penn MVSEC

from importPennMvsec import importPennMvsecDavis, importPennMvsecGt 


filePathOrName = os.path.join(prefix, 'data/mvsec/indoor_flying1_data.bag')
'''
Optionally, override the default template ...
template = {
    'left': {
        'dvs': '/davis/left/events',
        'frame': '/davis/left/image_raw',
        'imu': '/davis/left/imu',
        'cam': '/davis/left/camera_info',
    }, 'right': {
        'dvs': '/davis/right/events',
        'frame': '/davis/right/image_raw',
        'imu': '/davis/right/imu',
        'cam': '/davis/right/camera_info',
        }
    }
'''
importedDavis = importPennMvsecDavis(filePathOrName=filePathOrName)

filePathOrName = os.path.join(prefix, 'data/mvsec/indoor_flying1_gt.bag')
importedGt = importPennMvsecGt(filePathOrName=filePathOrName)

#imported = [importedDavis, importedGt]

# Rafactor into single container
container = importedDavis
container['data'].update(importedGt['data'])


#%% Import realsense

from importIntelRealsense import importIntelRealsense
    
filePathOrName = os.path.join(prefix, '/data/2019_10_23_static/20191023_165520.bag')

imported = importIntelRealsense(filePathOrName=filePathOrName)

#%% MANIPULATION FUNCTIONS

#%% Cropping a dataset to a desired time range

from split import cropTime

cropped = cropTime(imported, minTime=35, maxTime=38.5)

#%% Cropping a dataset to a desired spatial range
# works for dvs and derived data types 2020_01 doesn't yet work for frame datatype

from split import cropSpace

# This example takes all events with x in 9-19 inclusive, and with y in 0-9 inclusive
cropped = cropSpace(imported, minX=9, maxX=19, maxY= 9)

#%% Splitting a dataset by labels

from split import splitByLabel, selectByLabel

# select your labelled data from an import (alternatively label it using some processing)
labelledData = imported['data']['right']['dvslbl']

splitData = splitByLabel(labelledData, 'lbl') # 'lbl' in this case is the name of the field that contains the labels

# Alternatively, select a single label

selectedData = selectByLabel(labelledData, 'lbl', 3) # in this case, 3 is the label you want to isolate

#%% VISUALISATION FUNCTIONS

#%% Info

from info import info

info(imported)

#%% Timestamp info only

from info import infoTsForImportedDicts

infoTsForImportedDicts(imported)

#%% General function for plotting data present in channels according to its type

from plot import plot

plot(imported, zeroT=True, polarised=True)

#%% More specific example of taking particular datatypes from an import and 
# visualising them

from plotDvsContrast import plotDvsContrast
from plotFrame import plotFrame
from plotPose import plotPose

# Unpack
cam = imported['data']['davis']['cam']
events = imported['data']['davis']['dvs']
frames = imported['data']['davis']['frame']
poses = imported['data']['extra']['pose6q']
if 'frame' in imported['data']['extra']:
    depthMaps = imported['data']['extra']['frame']
else:
    depthMaps = None
    
plotDvsContrast(events, numPlots=2)
plotFrame(frames, numPlots=15)
plotPose(poses)
# Using 'frame' datatype to import and visualise depth maps
if depthMaps:
    plotFrame(depthMaps, numPlots=2) 


#%% Distribute the sub-plots in a visualisation of dvs event images 
# by event density rather than time

from plotDvsContrast import plotDvs

plotDvs(imported, numPlots=6, distributeBy='events')

#%% This function for visualising dvs surface-of-active-events-(SAE)-like
# visualisation is not included in standard plot function.

from plotDvsLastTs import plotDvsLastTs

plotDvsLastTs(imported)

#%% EXPORT FUNCTIONS

#%% Export to yarp

from exportIitYarp import exportIitYarp

exportIitYarp(imported, 
              exportFilePath= 'C:/data/mvsec/indoorFlying1Yarp', 
              pathForPlayback= '/home/sbamford/data/mvsec/indoorFlying1Yarp')


#%% Choose to export only specific datatypes; 
# overwrite data if the export is already there

from exportIitYarp import exportIitYarp

exportIitYarp(imported, 
              exportFilePath= 'C:/data/rpg/shapes_rotation', 
              pathForPlayback= '/home/sbamford/data/rpg/shapes_rotation', 
              dataTypes = ['imu', 'dvs'],
              overwrite=True)
