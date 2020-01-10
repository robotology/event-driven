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

# Optional dependency, for advanced visualisation ...
try:
    from mayavi import mlab
except ModuleNotFoundError:
    pass # catch it later

# Local imports
from plotDvsContrast import getEventImage


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

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        kwargs['startTime'] = time - time_window/2
        kwargs['stopTime'] = time + time_window/2
        image = getEventImage(data, **kwargs)
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image

    def get_dims(self):
        data = self.__data
        x = data['dimX'] if 'dimX' in data else np.max(self.data['x']) + 1
        y = data['dimY'] if 'dimY' in data else np.max(self.data['y']) + 1
        return x, y

    
class VisualiserFrame(Visualiser):

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        frameIdx = np.searchsorted(data['ts'], time)
        image = self.data['frames'][frameIdx]
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
    
    def get_dims(self):        
        data = self.__data
        x = data['dimX'] if 'dimX' in data else data['frames'][0].shape[1]
        y = data['dimY'] if 'dimY' in data else data['frames'][0].shape[0]
        return x, y


def quat2RotM(quat, M=None):
    if M is None: 
        M = np.zeros((4, 4))
    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]
    M[0, 0] = 1 - 2*y**2 - 2*z**2
    M[0, 1] = 2*x*y - 2*z*w
    M[0, 2] = 2*x*z + 2*y*w
    M[1, 0] = 2*x*y + 2*z*w
    M[1, 1] = 1 - 2*x**2 - 2*z**2
    M[1, 2] = 2*y*z - 2*x*w
    M[2, 0] = 2*x*z - 2*y*w
    M[2, 1] = 2*y*z + 2*x*w
    M[2, 2] = 1 - 2*x**2 - 2*y**2
    M[3, 3] = 1

def rotateUnitVectors(rotM):
    xVec = np.expand_dims(np.array([1, 0, 0, 1]), axis=1)
    yVec = np.expand_dims(np.array([0, 1, 0, 1]), axis=1)
    zVec = np.expand_dims(np.array([0, 0, 1, 1]), axis=1)
    allVecs = np.concatenate((xVec, yVec, zVec), axis=1)
    return rotM.dot(allVecs)

def project3dTo2d(X, Y, Z, smallestRenderDim):
    projX = X / Z
    projY = Y / Z
    projX = (projX + 1) * smallestRenderDim / 2
    projY = (projY + 1) * smallestRenderDim / 2
    projX = projX.astype(int)
    projY = projY.astype(int)
    return projX, projY

class VisualiserPose6q(Visualiser):

    renderX = 300
    renderY = 300
        
    def set_data(self, data):
        self.__data = data
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
        smallestRenderDim = min(self.renderX, self.renderY)
        self.projX, self.projY = project3dTo2d(poseX, poseY, poseZ, smallestRenderDim)
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
        self.projX_X, self.projY_X = project3dTo2d(poseX_X, poseY_X, poseZ_X, smallestRenderDim)
        self.projX_Y, self.projY_Y = project3dTo2d(poseX_Y, poseY_Y, poseZ_Y, smallestRenderDim)
        self.projX_Z, self.projY_Z = project3dTo2d(poseX_Z, poseY_Z, poseZ_Z, smallestRenderDim)
    
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        poseIdx = np.searchsorted(data['ts'], time)
        image = np.zeros((self.renderX, self.renderY), dtype = np.uint8)
        image[self.projX[poseIdx], self.projY[poseIdx]] = 255
        image[self.projX_X[poseIdx], self.projY_X[poseIdx]] = 255
        image[self.projX_Y[poseIdx], self.projY_Y[poseIdx]] = 255
        image[self.projX_Z[poseIdx], self.projY_Z[poseIdx]] = 255
        # Allow for arbitrary post-production on image with a callback
        # TODO: as this is boilerplate, it could be pushed into pie syntax ...
        if kwargs.get('callback', None) is not None:
            kwargs['image'] = image
            image = kwargs['callback'](**kwargs)
        return image
    
    def get_dims(self):        
        return self.renderX, self.renderY

