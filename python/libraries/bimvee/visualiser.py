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
    return M

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

# Spherical linear interpolation, adapted from https://en.wikipedia.org/wiki/Slerp
DOT_THRESHOLD = 0.9995
def slerp(q1, q2, time_relative):
    dot = np.sum(q1 * q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > DOT_THRESHOLD:
        result = q1 + time_relative * (q2 - q1)
        return (result.T / np.linalg.norm(result, axis=1)).T
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * time_relative
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q1) + (s1 * q2)

# adapted from https://stackoverflow.com/questions/50387606/python-draw-line-between-two-coordinates-in-a-matrix
def draw_line(mat, x0, y0, x1, y1):
    if not (0 <= x0 < mat.shape[0] and 0 <= x1 < mat.shape[0] and
            0 <= y0 < mat.shape[1] and 0 <= y1 < mat.shape[1]):
        print('Invalid coordinates.')
        return
    if (x0, y0) == (x1, y1):
        mat[x0, y0] = 2
        return
    # Swap axes if Y slope is smaller than X slope
    transpose = abs(x1 - x0) < abs(y1 - y0)
    if transpose:
        mat = mat.T
        x0, y0, x1, y1 = y0, x0, y1, x1
    # Swap line direction to go left-to-right if necessary
    if x0 > x1:
        x0, y0, x1, y1 = x1, y1, x0, y0
    # Compute intermediate coordinates using line equation
    x = np.arange(x0, x1 + 1)
    y = np.round(((y1 - y0) / (x1 - x0)) * (x - x0) + y0).astype(x.dtype)
    # Write intermediate coordinates
    mat[x, y] = 255

class VisualiserPose6q(Visualiser):

    renderX = 300
    renderY = 300

    '''
    Offset and scale the pose translations so that they all fit into the volume:
        x [-0.5:0.5]
        y[-0.5:0.5]
        z[1:2]
    ''' 
    def centreAndScalePoses(self):
        data = self.__data
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
        data['pose'][:, 0] = poseX
        data['pose'][:, 1] = poseY
        data['pose'][:, 2] = poseY
                
    def set_data(self, data):
        self.__data = data
        self.smallestRenderDim = min(self.renderX, self.renderY)
        #self.centreAndScalePoses()
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