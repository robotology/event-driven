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
importIntelRealsense is compatible with importAe function for intel realsense (t265).
The output of the realsense is a .bag file. 
This function is just a wrapper for the importRpgDvsRos function, 
which applies a default template to get the pose data out to ch0 - pose6q. 
"""

#%%

from importRpgDvsRos import importRpgDvsRos
    
def importIntelRealsense(**kwargs):
    if kwargs.get('template') is None:
        kwargs['template'] = {
                'ch0': {
                        'pose6q': '/device_0/sensor_0/Pose_0/pose/transform/data',
                        }
                }
        kwargs['poseAsTransform'] = True
    return importRpgDvsRos(**kwargs)
