# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
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
plotPose takes 'inDict' - a dictionary containing imported pose data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and plots against time the various dimensions of the 
imu samples contained. 
plotTrajectory

Also handles point3 type, which contains 3d points
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def plotPose(inDict, **kwargs):
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotPose(inDictInst, **kwargs)
        return
    if 'info' in inDict:
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotPose was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'pose6q' in channelData and len(channelData['pose6q']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotPose(channelData['pose6q'], **kwargs)
            if 'point3' in channelData and len(channelData['point3']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotPose(channelData['point3'], **kwargs)
            if 'pose6q' not in channelData and 'point3' not in channelData:
                print('Channel ' + channelName + ' skipped because it contains no pose data')
        return    

    if 'rotation' in inDict:
        fig, allAxes = plt.subplots(2, 1)
        axesT = allAxes[0]
        axesR = allAxes[1]
        axesR.plot(inDict['ts'], inDict['rotation'])
        axesR.legend(['r_w', 'r_x', 'r_y', 'r_z'])
    if 'point' in inDict:
        if not 'axesT' in locals():
            fig, axesT = plt.subplots()
        if kwargs.get('zeroT', False):
            point = np.copy(inDict['point'])
            firstPoint = inDict['point'][0, :]
            point = point - firstPoint
        else:
            point = inDict['point']    
        axesT.plot(inDict['ts'], point)
        axesT.legend(['x', 'y', 'z'])
    # TODO: Callbacks
    
"""
Plot the 3D trajectory of a body or marker for the entire duration of recording

To select which bodies to plot using bodyID: 
 - look at the bodyIDs parsed using importVicon
 - pass a list of strings present in the bodyID of choice, through the parameter include
 - pass a list of strings that should be absent from the bodyID of choice, through the parameter exclude
"""

def plotTrajectories(viconDataDict, bodyIds, include, exclude, **kwargs):
    ax = kwargs.get('ax')
    if ax is None:
        ax = plt.axes(projection='3d')
        kwargs['ax'] = ax
    for name in bodyIds:
        select_body = all([(inc in name) for inc in include]) and all([not (exc in name) for exc in exclude])
        if select_body:  # modify this line to plot whichever markers you want
            marker_pose = viconDataDict['data'][name]['pose6q']['point']
            ax.scatter3D(marker_pose[:, 0], marker_pose[:, 1], marker_pose[:, 2], label=name)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = ax # TODO: make this handling consistent across the library
        callback(**kwargs)
    #return ax # ax is also available to the calling function inside **kwargs

    
    
