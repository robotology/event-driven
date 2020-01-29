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
if __package__ is None or __package__ == '':
    from plotDvsContrast import getEventImageForTimeRange
    from geometry import quat2RotM, rotateUnitVectors, project3dTo2d, slerp, draw_line
    from split import splitByLabel
else:
    # This format allows for a certain configuration of the ntupleviz visualiser, maybe?
    from .plotDvsContrast import getEventImageForTimeRange
    from .geometry import quat2RotM, rotateUnitVectors, project3dTo2d, slerp, draw_line
    from .split import splitByLabel

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
    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        kwargs['startTime'] = time - timeWindow/2
        kwargs['stopTime'] = time + timeWindow/2
        kwargs['dimX'] = data['dimX']
        kwargs['dimY'] = data['dimY']
        image = getEventImageForTimeRange(data, **kwargs)
        # Post processing to get image into uint8 with correct scale
        contrast = kwargs.get('contrast', 3)
        if kwargs.get('polarised', (kwargs.get('polarized'), True)):
            image = ((image + contrast) / contrast / 2 * 255).astype(np.uint8)
        else:
            image = (image / contrast * 255).astype(np.uint8)
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

    def set_data(self, data):
        self.__data = data
       
        if data['frames'][0].dtype != np.uint8:
            data['frames'] = [(frame*255).astype(np.uint8) for frame in data['frames']] #TODO: assuming that it starts scaled in 0-1 - could have more general approach?
    
    def get_colorfmt(self):
        try:
            if len(self.__data['frames'][0].shape) == 3:
                return 'rgb'
            else:
                return 'luminance'
        except: # TODO None type error?
            return 'luminance'
            
    def get_default_image(self):
        x, y = self.get_dims()
        # Return an x,y,3 by default i.e. rgb, for safety, since in the absence of data we may not know how the texture's colorfmt is set
        return np.ones((x, y, 3), dtype=np.uint8) * 128 # TODO: Hardcoded midway (grey) value
        

    # TODO: There can be methods which better choose the best frame, or which create a visualisation which
    # respects the time_window parameter 
    def get_frame(self, time, timeWindow, **kwargs):
        data = self.__data
        if 'tsEnd' in data:
            # Optional mode in which frames are only displayed 
            # between corresponding ts and tsEnd
            frameIdx = np.searchsorted(data['ts'], time, side='right') - 1
            if frameIdx < 0:
                image = self.get_default_image()
            elif time > data['tsEnd'][frameIdx]:
                image = self.get_default_image()
            else:                
                image = data['frames'][frameIdx]
        elif time < data['ts'][0] - timeWindow / 2 or time > data['ts'][-1] + timeWindow / 2:
            # Gone off the end of the frame data
            image = self.get_default_image()
        else:
            frameIdx = find_nearest(data['ts'], time)
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

    renderX = 300 # TODO Hardcoded
    renderY = 300
    labels = None

    def __init__(self, data):
        self.set_data(data)
        self.smallestRenderDim = min(self.renderX, self.renderY)

    '''
    Offset and scale the pose translations so that they all fit into the volume:
        x [-0.5:0.5]
        y[-0.5:0.5]
        z[1:2]
    '''
    def set_data(self, data):
        if 'bodyId' in data:
            # split pose data by label, for ease of reference during rendering
            self.__data = splitByLabel(data, 'bodyId')
            self.labels = np.unique(data['bodyId'])
        else:
            self.__data = [data]
            
        pointX = data['point'][:, 0]
        pointY = data['point'][:, 1]
        pointZ = data['point'][:, 2]
        minX = np.min(pointX)
        maxX = np.max(pointX)
        minY = np.min(pointY)
        maxY = np.max(pointY)
        minZ = np.min(pointZ)
        maxZ = np.max(pointZ)
        self.centreX = (minX + maxX) / 2
        self.centreY = (minY + maxY) / 2
        self.centreZ = (minZ + maxZ) / 2
        self.largestDim = max(maxX-minX, maxY-minY, maxZ-minZ)
        if self.largestDim == 0:
            self.largestDim = 1

    def project_pose(self, point, rotation, image):
        if point is None:
            return image
        # Centre the pose, unpacking it at the same time
        pointX = (point[0] - self.centreX) / self.largestDim
        pointY = (point[1] - self.centreY) / self.largestDim
        pointZ = (point[2] - self.centreZ) / self.largestDim + 2        
        # Project the location
        projX, projY = project3dTo2d(pointX, pointY, pointZ, self.smallestRenderDim)
        if rotation is None:
            # Just put a dot where the point should be
            image[projY, projX, :] = 255
        else:
            rotMats = quat2RotM(rotation)
            rotatedUnitVectors = rotateUnitVectors(rotMats)    
            pointX_X = pointX + rotatedUnitVectors[0, 0]
            pointX_Y = pointX + rotatedUnitVectors[1, 0]
            pointX_Z = pointX + rotatedUnitVectors[2, 0]
            pointY_X = pointY + rotatedUnitVectors[0, 1]
            pointY_Y = pointY + rotatedUnitVectors[1, 1]
            pointY_Z = pointY + rotatedUnitVectors[2, 1]
            pointZ_X = pointZ + rotatedUnitVectors[0, 2]
            pointZ_Y = pointZ + rotatedUnitVectors[1, 2]
            pointZ_Z = pointZ + rotatedUnitVectors[2, 2]
            projX_X, projY_X = project3dTo2d(pointX_X, pointY_X, pointZ_X, self.smallestRenderDim)
            projX_Y, projY_Y = project3dTo2d(pointX_Y, pointY_Y, pointZ_Y, self.smallestRenderDim)
            projX_Z, projY_Z = project3dTo2d(pointX_Z, pointY_Z, pointZ_Z, self.smallestRenderDim)
            draw_line(image[:, :, 0], projX, projY, projX_X, projY_X)
            draw_line(image[:, :, 1], projX, projY, projX_Y, projY_Y)
            draw_line(image[:, :, 2], projX, projY, projX_Z, projY_Z)
        return image
    
    def get_frame(self, time, timeWindow, **kwargs):
        allData = self.__data
        if allData is None:
            print('Warning: data is not set')
            return np.zeros((1, 1), dtype=np.uint8) # This should not happen
        image = np.zeros((self.renderX, self.renderY, 3), dtype = np.uint8)
        for data in allData:
            # Note, for the following, this follows library function pose6qInterp, but is broken out here, because of slight behavioural differences.
            idxPre = np.searchsorted(data['ts'], time, side='right') - 1
            timePre = data['ts'][idxPre]
            if timePre == time:
                # In this edge-case of desired time == timestamp, there is no need 
                # to interpolate 
                point = data['point'][idxPre, :]
                rotation = data['rotation'][idxPre, :]
            elif idxPre < 0 or (idxPre >= len(data['ts'])-1):
                # In this edge-case of the time at the beginning or end, 
                # don't show any pose
                point = None
                rotation = None
            else:
                if kwargs.get('interpolate', True):
                    timePost = data['ts'][idxPre + 1]
                    qPre = data['rotation'][idxPre, :]
                    qPost = data['rotation'][idxPre + 1, :]
                    timeRel = (time - timePre) / (timePost - timePre)
                    rotation = slerp(qPre, qPost, timeRel)
                    locPre = data['point'][idxPre, :] 
                    locPost = data['point'][idxPre + 1, :]
                    point = locPre * (1-timeRel) + locPost * timeRel
                    timeDist = min(time - timePre, timePost - time)
                    if timeDist > timeWindow / 2:
                        # Warn the viewer that this interpolation is 
                        # based on data beyond the timeWindow
                        image[:30,:30,0] = 255
                else: # No interpolation, so just choose the sample which is nearest in time
                    poseIdx = find_nearest(data['ts'], time)
                    point = data['point'][poseIdx, :]
                    rotation = data['rotation'][poseIdx, :]
                
            image = self.project_pose(point, rotation, image)                
        
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