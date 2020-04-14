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
prefix = 'C:/' if os.name == 'nt' else '/home/sbamford/'    

# Add paths
sys.path.append(os.path.join(prefix, 'repos/event-driven/python'))
sys.path.append(os.path.join(prefix, 'repos/event-driven/python/libraries'))

import visualizer.ntupleviz

# Create the ntupleViz app and start it in a thread
visualizerApp = visualizer.ntupleviz.Ntupleviz()
thread = threading.Thread(target=visualizerApp.run)
thread.daemon = True
thread.start()

# Wait until the load dialog has come up

#%% Load some data that you want to work with

from importIitYarp import importIitYarp

filePathOrName = os.path.join(prefix, "data/2019_11_11_AikoImu/linear_100/ATIS")
container = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

# Having loaded a dvs dataDict - poke it into the dsm
visualizerApp.root.data_controller.data_dict = imported

#%% Load some different data and push it in

from importIitYarp import importIitYarp

filePathOrName = os.path.join(prefix, "data/2019_11_11_AikoImu/tripod_pitch/ATIS")
container = importIitYarp(filePathOrName=filePathOrName, tsBits=30)

visualizerApp.root.data_controller.data_dict = container

#%% Data from SECDVS

from bimvee.importSecDvs import importSecDvs

filePathOrName = os.path.join(prefix, "data/2020_03_23 SecDvs from Ander/2020-03-24-12-45-00.bin")
container = importSecDvs(filePathOrName=filePathOrName)

# Having loaded a dvs dataDict - poke it into the dsm
visualizerApp.root.data_controller.data_dict = container

#%% Simulated DAVIS data

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/simulation_3walls.bag')

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

container = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

visualizerApp.root.data_controller.data_dict = container

#%% Experiment with pose interpolation

pose = container['data']['extra']['pose6q']
toKeep = [0, 300, 600, 900, 1200, 1500, 1800, 1999]
poseKept = {} 
poseKept['ts'] = pose['ts'][toKeep]
poseKept['point'] = pose['point'][toKeep, :]
poseKept['rotation'] = pose['rotation'][toKeep, :]

container['data']['reduced'] = {'pose6q': poseKept}

visualizerApp.root.data_controller.data_dict = container

#%% Real DAVIS data

# http://rpg.ifi.uzh.ch/davis_data.html
from importRpgDvsRos import importRpgDvsRos
    
filePathOrName = os.path.join(prefix, 'data/rpg/shapes_translation.bag')

template = {
    'davis': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        'imu': '/dvs/imu',
        },
    'extra': {
        'pose6q': '/optitrack/davis'
        }
    }

container = importRpgDvsRos(filePathOrName=filePathOrName, template=template, )

visualizerApp.root.data_controller.data_dict.data_dict = container

#%% Constructed rolling poses - X

import numpy as np

ts = np.array([0, 1, 2, 3, 4], dtype=np.float64)
point = np.zeros((5, 3), dtype=np.float64)
# from neutral pose, to x = 1 (i.e. roll backwards through 180) 
# back to neutral, then x to -1 ( i.e. roll forwards head over heels through 180)
# back to neutral 
rotation = np.array([[1, 0, 0, 0], 
                     [0, 1, 0, 0], 
                     [1, 0, 0, 0], 
                     [0, -1, 0, 0], 
                     [1, 0, 0, 0]],
                    dtype=np.float64)

trialContainer = {'pose6q': {'ts': ts,
                        'point': point,
                        'rotation': rotation
                        }
            }

visualizerApp.root.data_controller.data_dict.data_dict = trialContainer

#%% Constructed rolling poses - Y

import numpy as np

ts = np.array([0, 1, 2, 3, 4], dtype=np.float64)
point = np.zeros((5, 3), dtype=np.float64)
# from neutral pose, to y = 1 (i.e. swivel clockwise through 180 as if on an office chair) 
# back to neutral, then y to -1 ( i.eswivel anticlockwise through 180)
# back to neutral 
rotation = np.array([[1, 0, 0, 0], 
                     [0, 0, 1, 0], 
                     [1, 0, 0, 0], 
                     [0, 0, -1, 0], 
                     [1, 0, 0, 0]],
                    dtype=np.float64)

trialContainer = {'pose6q': {'ts': ts,
                        'point': point,
                        'rotation': rotation
                        }
            }

visualizerApp.root.data_controller.data_dict.data_dict = trialContainer

#%% Constructed rolling poses - Z

import numpy as np

ts = np.array([0, 1, 2, 3, 4], dtype=np.float64)
point = np.zeros((5, 3), dtype=np.float64)
# from neutral pose, to z = 1 (i.e. roll our head clockwise through 180, whilst still looking forwards) 
# back to neutral, then z to -1 ( i.e. roll our head anticlockwise through 180, whilst still looking forwards)
# back to neutral 
rotation = np.array([[1, 0, 0, 0], 
                     [0, 0, 0, 1], 
                     [1, 0, 0, 0], 
                     [0, 0, 0, -1], 
                     [1, 0, 0, 0]],
                    dtype=np.float64)

trialContainer = {'pose6q': {'ts': ts,
                        'point': point,
                        'rotation': rotation
                        }
            }

visualizerApp.root.data_controller.data_dict.data_dict = trialContainer
