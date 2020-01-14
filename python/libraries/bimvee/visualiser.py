# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Sim Bamford
Code contributions from Massimiliano Iacono - contains classes which substitutes DualStreamManager class.

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Functionality for serving images which represent the data at a certain time
(or given a certain time window). 
The intended use case is to support video-like playback of data
There is a generic Visualiser class - it contains a dataType dict. 
This is subclassed for each supported dataType (e.g. dvs, frame, pose etc)
Each subclass should implement basic methods: get_dims, get_frame, get_colorfmt
Implementing set_data allows the visualiser to do some preparation for visualisation
when it receives new data

colorfmt is a choice between luminance and rgb. 
If luminance, then the frame returned should have dim 2 = 3.
Nothing stops the calling function from applying a color mask to an output in luminance format.      
"""

import numpy as np
from geometry import quat2RotM, rotateUnitVectors, project3dTo2d, slerp, draw_line

# Local imports
from plotDvsContrast import getEventImageForTimeRange


class Visualiser():
    
    def __init__(self, data):
        self.set_data(data)

    def set_data(self, data):
        self.__data = data
        
    def get_frame(self, time, timeWindow, **kwargs):
        return np.zeros((1, 1), dtype=np.uint8)

    def get_colorfmt(self):
        return 'luminance'


class VisualiserDvs(Visualiser):

    def __init__(self, data):
        self.set_data(data)

    def set_data(self, data):
        self.__data = data
        
    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        kwargs['startTime'] = time - time_window/2
        kwargs['stopTime'] = time + time_window/2
        image = getEventImageForTimeRange(data, **kwargs)
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image

    def get_dims(self):
        try:
            data = self.__data
        except AttributeError: # data hasn't been set yet
            return 1, 1
        x = data['dimX'] if 'dimX' in data else np.max(data['x']) + 1
        y = data['dimY'] if 'dimY' in data else np.max(data['y']) + 1
        return x, y

    
class VisualiserFrame(Visualiser):

    def __init__(self, data):
        self.set_data(data)

    def set_data(self, data):
        self.__data = data
        

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        frameIdx = np.searchsorted(data['ts'], time)
        if time < data['ts'][0] or time > data['ts'][-1]:
            # Gone off the end of the frame data
            image = np.ones(self.get_dims(), dtype=np.uint8) * 128 # TODO: Hardcoded midway (grey) value
        else:
            image = data['frames'][frameIdx]
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
    
    def get_dims(self):
        try:
            data = self.__data
        except AttributeError: # data hasn't been set yet
            return 1, 1
        x = data['dimX'] if 'dimX' in data else data['frames'][0].shape[1]
        y = data['dimY'] if 'dimY' in data else data['frames'][0].shape[0]
        return x, y

class VisualiserPose6q(Visualiser):

    renderX = 300
    renderY = 300

    '''
    Offset and scale the pose translations so that they all fit into the volume:
        x [-0.5:0.5]
        y[-0.5:0.5]
        z[1:2]
    ''' 
                
    def set_data(self, data):
        self.__data = data
        self.smallestRenderDim = min(self.renderX, self.renderY)
        poseX = data['pose'][:, 0]
        poseY = data['pose'][:, 1]
        poseZ = data['pose'][:, 2]
        minX = np.min(poseX)
        maxX = np.max(poseX)
        minY = np.min(poseY)
        maxY = np.max(poseY)
        minZ = np.min(poseZ)
        maxZ = np.max(poseZ)
        centreX = (minX + maxX) / 2
        centreY = (minY + maxY) / 2
        centreZ = (minZ + maxZ) / 2
        largestDim = max(maxX-minX, maxY-minY, maxZ-minZ)
        # Centre the poses
        poseX = (poseX - centreX) / largestDim
        poseY = (poseY - centreY) / largestDim
        poseZ = (poseZ - centreZ) / largestDim + 2
        
        self.projX, self.projY = project3dTo2d(poseX, poseY, poseZ, self.smallestRenderDim)
        #
        rotMats = np.apply_along_axis(quat2RotM, axis=1, arr=data['pose'][:, 3:7])    
        rotatedUnitVectors = np.apply_along_axis(rotateUnitVectors, axis=1, arr=rotMats)    
        poseX_X = poseX + rotatedUnitVectors[:, 0, 0]
        poseX_Y = poseX + rotatedUnitVectors[:, 1, 0]
        poseX_Z = poseX + rotatedUnitVectors[:, 2, 0]
        poseY_X = poseY + rotatedUnitVectors[:, 0, 1]
        poseY_Y = poseY + rotatedUnitVectors[:, 1, 1]
        poseY_Z = poseY + rotatedUnitVectors[:, 2, 1]
        poseZ_X = poseZ + rotatedUnitVectors[:, 0, 2]
        poseZ_Y = poseZ + rotatedUnitVectors[:, 1, 2]
        poseZ_Z = poseZ + rotatedUnitVectors[:, 2, 2]
        self.projX_X, self.projY_X = project3dTo2d(poseX_X, poseY_X, poseZ_X, self.smallestRenderDim)
        self.projX_Y, self.projY_Y = project3dTo2d(poseX_Y, poseY_Y, poseZ_Y, self.smallestRenderDim)
        self.projX_Z, self.projY_Z = project3dTo2d(poseX_Z, poseY_Z, poseZ_Z, self.smallestRenderDim)
    
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        poseIdx = np.searchsorted(data['ts'], time)
        image = np.zeros((self.renderX, self.renderY, 3), dtype = np.uint8)
        if kwargs.get('interpolate', False):
            # If interpolation is desired we reject the precalculated projections
            pass
        else:
            X = self.projX[poseIdx]
            Y = self.projY[poseIdx]
            X_X = self.projX_X[poseIdx]
            Y_X = self.projY_X[poseIdx]
            X_Y = self.projX_Y[poseIdx]
            Y_Y = self.projY_Y[poseIdx]
            X_Z = self.projX_Z[poseIdx]
            Y_Z = self.projY_Z[poseIdx]
            draw_line(image[:, :, 0], X, Y, X_X, Y_X)
            draw_line(image[:, :, 1], X, Y, X_Y, Y_Y)
            draw_line(image[:, :, 2], X, Y, X_Z, Y_Z)

        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
    
    def get_dims(self):        
        return self.renderX, self.renderY

    def get_colorfmt(self):
        return 'rgb'