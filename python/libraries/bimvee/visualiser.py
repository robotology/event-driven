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
when it receives new data.

In general get_frame takes two args: time, and time_window.
In general one could think about representing data in an interpolated way or not.
For example, with poses, imu etc, one could interpolate between samples, 
or one could simply choose the sample which is nearest in time.
Likewise for frames. 
The time_window parameter says how much data to take around the sample point for event-type data.
It might be possible to develop visualisations for other types of data that make use of the concept of a time window. 

colorfmt is a choice between luminance and rgb. 
If luminance, then the frame returned should have dim 2 = 3.
Nothing stops the calling function from applying a color mask to an output in luminance format.      
"""

import numpy as np
import math

# Local imports
try:
    from plotDvsContrast import getEventImageForTimeRange
    from geometry import quat2RotM, rotateUnitVectors, project3dTo2d, slerp, draw_line
except ImportError:
    # This format allows for a certain configuration of the ntupleviz visualiser, maybe?
    from libraries.bimvee.plotDvsContrast import getEventImageForTimeRange
    from libraries.bimvee.geometry import quat2RotM, rotateUnitVectors, project3dTo2d, slerp, draw_line

# A function intended to find the nearest timestamp
# adapted from https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array
def find_nearest(array, value):
    idx = np.searchsorted(array, value) # side="left" param is the default
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return idx-1
    else:
        return idx

class Visualiser():
    
    __data = None
    
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
        kwargs['dimX'] = data['dimX']
        kwargs['dimY'] = data['dimY']
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
        if 'dimX' in data:
            x = data['dimX'] 
        else:
            x = np.max(data['x']) + 1
            data['dimX'] = x
        if 'dimY' in data:
            y = data['dimY'] 
        else:
            y = np.max(data['y']) + 1
            data['dimY'] = y
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
        frameIdx = find_nearest(data['ts'], time)
        if time < data['ts'][0] - time_window / 2 or time > data['ts'][-1] + time_window / 2:
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
        self.centreX = (minX + maxX) / 2
        self.centreY = (minY + maxY) / 2
        self.centreZ = (minZ + maxZ) / 2
        self.largestDim = max(maxX-minX, maxY-minY, maxZ-minZ)
        # Centre the poses
        poseX = (poseX - self.centreX) / self.largestDim
        poseY = (poseY - self.centreY) / self.largestDim
        poseZ = (poseZ - self.centreZ) / self.largestDim + 2
        
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

    def project_pose(self, poseX, poseY, poseZ, poseQ, image):
        # Centre the pose, unpacking it at the same time
        poseX = (poseX - self.centreX) / self.largestDim
        poseY = (poseY - self.centreY) / self.largestDim
        poseZ = (poseZ - self.centreZ) / self.largestDim + 2        
        # Project the location
        projX, projY = project3dTo2d(poseX, poseY, poseZ, self.smallestRenderDim)
        rotMats = quat2RotM(poseQ)
        rotatedUnitVectors = rotateUnitVectors(rotMats)    
        poseX_X = poseX + rotatedUnitVectors[0, 0]
        poseX_Y = poseX + rotatedUnitVectors[1, 0]
        poseX_Z = poseX + rotatedUnitVectors[2, 0]
        poseY_X = poseY + rotatedUnitVectors[0, 1]
        poseY_Y = poseY + rotatedUnitVectors[1, 1]
        poseY_Z = poseY + rotatedUnitVectors[2, 1]
        poseZ_X = poseZ + rotatedUnitVectors[0, 2]
        poseZ_Y = poseZ + rotatedUnitVectors[1, 2]
        poseZ_Z = poseZ + rotatedUnitVectors[2, 2]
        projX_X, projY_X = project3dTo2d(poseX_X, poseY_X, poseZ_X, self.smallestRenderDim)
        projX_Y, projY_Y = project3dTo2d(poseX_Y, poseY_Y, poseZ_Y, self.smallestRenderDim)
        projX_Z, projY_Z = project3dTo2d(poseX_Z, poseY_Z, poseZ_Z, self.smallestRenderDim)
        draw_line(image[:, :, 0], projX, projY, projX_X, projY_X)
        draw_line(image[:, :, 1], projX, projY, projX_Y, projY_Y)
        draw_line(image[:, :, 2], projX, projY, projX_Z, projY_Z)
        return image
    
    def get_frame(self, time, time_window, **kwargs):
        data = self.__data
        if data is None:
            print('YUP that happened')
            return np.zeros((1, 1), dtype=np.uint8) # This should not happen
        image = np.zeros((self.renderX, self.renderY, 3), dtype = np.uint8)
        if kwargs.get('interpolate', True):
            # If interpolation is desired we reject the precalculated projections
            idxPre = np.searchsorted(data['ts'], time, side='right') - 1
            timePre = data['ts'][idxPre]
            if timePre == time:
                # In this edge-case of desired time == timestamp, there is no need 
                # to interpolate - just use a precalculated projection
                X = self.projX[idxPre]
                Y = self.projY[idxPre]
                X_X = self.projX_X[idxPre]
                Y_X = self.projY_X[idxPre]
                X_Y = self.projX_Y[idxPre]
                Y_Y = self.projY_Y[idxPre]
                X_Z = self.projX_Z[idxPre]
                Y_Z = self.projY_Z[idxPre]
                draw_line(image[:, :, 0], X, Y, X_X, Y_X)
                draw_line(image[:, :, 1], X, Y, X_Y, Y_Y)
                draw_line(image[:, :, 2], X, Y, X_Z, Y_Z)
            elif idxPre < 0 or idxPre >= len(data['ts']):
                # In this edge-case of the time at the beginning or end, 
                # don't show any pose
                pass
            else:
                timePost = data['ts'][idxPre + 1]
                qPre = data['pose'][idxPre, 3:7]
                qPost = data['pose'][idxPre + 1, 3:7]
                timeRel = (time - timePre) / (timePost - timePre)
                qInterp = slerp(qPre, qPost, timeRel)
                locPre = data['pose'][idxPre, 0:3] 
                locPost = data['pose'][idxPre + 1, 0:3]
                locInterp = locPre * (1-timeRel) + locPost * timeRel
                image = self.project_pose(locInterp[0], locInterp[1], locInterp[2], qInterp, image)                
        else: # No interpolation, so just choose the sample which is nearest in time
            poseIdx = find_nearest(data['ts'], time)
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