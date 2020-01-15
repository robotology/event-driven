# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Massimiliano Iacono
         Sim Bamford

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Examples of how to use "ntupleviz" visualiser to visualise imported data.
It's possible to use the visualizer as a standalone app, 
using its load dialog to get data into it. 
However this script assumes running the code section by section 
(sections delimited by #%%) as in an IDE such as pycharm or spyder. 
The visualiser is started in a thread, then data is pushed to it from the IDE.
This allows for a workflow in which data is progressively imported and manipulated
in the IDE, and inspected by the visualizer at each stage. 
Note, however, that if the visualiser is closed or crashes, it must be re-run 
from a fresh console, which may mean re-importing and processing data. 
"""

#%% Preliminaries:

''' 
Run this from a fresh console
Make sure you launch from within the dual_visualizer folder.
'''

import os, sys
import threading
#import numpy as np

# Get os-specific path for local libraries (YMMV)
if os.name == 'nt':
    prefix = 'C:/'
else:
    prefix = '/home/sbamford/'    

# Add path to bimvee library
sys.path.append(os.path.join(prefix, 'repos/event-driven-python-dev/python/libraries/bimvee'))
# Add path to visulaizer
sys.path.append(os.path.join(prefix, 'repos/event-driven-python-dev/python/visualizer'))

# The load dialog requires this in order to start up properly
#os.environ['HOME'] =  prefix

import ntupleviz

# Actually, let's enforce being in the right directory
#os.chdir(os.path.join(prefix, 'repos/dual-stream'))

# Create the dualViz app and start it in a thread
visualizerApp = ntupleviz.Ntupleviz()
thread = threading.Thread(target=visualizerApp.run)
thread.daemon = True
thread.start()

# Wait until the load dialog has come up

#%% Load some data that you want to work with

from importIitYarp import importIitYarp

filePathOrName = os.path.join(prefix, "data/2019_11_11_AikoImu/linear_100/ATIS")
imported = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

dvs = imported['data']['right']['dvs']

# Having loaded a dvs dataDict - poke it into the dsm
visualizerApp.root.data_controller.data_dict = imported

#%% Load some different data and push it in

from importIitYarp import importIitYarp

filePathOrName = os.path.join(prefix, "data/2019_11_11_AikoImu/tripod_pitch/ATIS")
imported = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

dvs = imported['data']['right']['dvs']

visualizerApp.root.data_controller.data_dict.data_dict = imported

#%% Simulated DAVIS data

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/simulation_3planes.bag')

template = {
    'davis': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        },
    'extra': {
        'frame': '/dvs/depthmap',
        'pose6q': '/dvs/pose'
        }
    }

imported = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

pose = imported['data']['extra']['pose6q']
dvs = imported['data']['davis']['dvs']
frame = imported['data']['davis']['frame']
depthmap = imported['data']['extra']['frame']

visualizerApp.root.data_controller.data_dict.data_dict = imported

#%% Real DAVIS data

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/shapes_rotation.bag')

template = {
    'davis': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        },
    'extra': {
        'pose6q': '/optitrack/davis'
        }
    }

imported = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

pose = imported['data']['extra']['pose6q']
dvs = imported['data']['davis']['dvs']
frame = imported['data']['davis']['frame']

visualizerApp.root.data_controller.data_dict.data_dict = imported



#%% Demonstration of pose interpolation

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/shapes_rotation.bag')

template = {
    'davis': {
        'frame': '/dvs/image_raw',
        },
    'extra': {
        'pose6q': '/optitrack/davis'
        }
    }

imported = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

pose = imported['data']['extra']['pose6q']
frame = imported['data']['davis']['frame']


keepIds = [0, 4000, 8000, 11882]

pose['ts'] = pose['ts'][keepIds]
pose['pose'] = pose['pose'][keepIds, :]

visualizerApp.root.data_controller.data_dict.data_dict = imported

