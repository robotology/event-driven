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

# local imports
if __package__ is None or __package__ == '':
    from split import cropTime
else:
    from .split import cropTime

def plotPose(inDicts, **kwargs):
    if isinstance(inDicts, list):
        for inDict in inDicts:
            plotPose(inDict, **kwargs)
        return
    else:
        inDict = inDicts
    if not isinstance(inDict, dict):
        return
    if 'ts' not in inDict:
        title = kwargs.pop('title', '')
        if 'info' in inDict and isinstance(inDict, dict):
            fileName = inDict['info'].get('filePathOrName')
            if fileName is not None:
                print('plotPose was called for file ' + fileName)
                title = (title + ' ' + fileName).lstrip()
        for key in inDict.keys():
            kwargs['title'] = (title + ' ' + key).lstrip()
            plotPose(inDict[key], **kwargs)
        return
    # From this point onwards, it's a data-type container
    if 'point' not in inDict:
        return
    # From this point onwards, it's a pose or point data-type container    

    ts = inDict['ts']
    minTime = kwargs.get('minTime', kwargs.get('startTime', kwargs.get('beginTime', ts.min())))
    maxTime = kwargs.get('maxTime', kwargs.get('stopTime', kwargs.get('endTime', ts.max())))
    if minTime > ts.min() or maxTime < ts.max():
        inDict = cropTime(inDict, minTime=minTime, maxTime=maxTime, zeroTime=False)
        ts = inDict['ts']

    if 'rotation' in inDict:
        fig, allAxes = plt.subplots(2, 1)
        fig.suptitle(kwargs.get('title', ''))
        axesT = allAxes[0]
        axesR = allAxes[1]
        rotation = inDict['rotation']
        axesR.plot(ts, rotation[:, 0], 'k')
        axesR.plot(ts, rotation[:, 1], 'r')
        axesR.plot(ts, rotation[:, 2], 'g')
        axesR.plot(ts, rotation[:, 3], 'b')
        axesR.legend(['r_w', 'r_x', 'r_y', 'r_z'])
        axesR.set_xlabel('Time (s)')
        axesR.set_ylabel('Quaternion components')
        axesR.set_ylim([-1, 1])
    if 'point' in inDict:
        if not 'axesT' in locals():
            fig, axesT = plt.subplots()
        if kwargs.get('zeroT', False):
            point = np.copy(inDict['point'])
            firstPoint = inDict['point'][0, :]
            point = point - firstPoint
        else:
            point = inDict['point']
        axesT.plot(ts, point[:, 0], 'r')
        axesT.plot(ts, point[:, 1], 'g')
        axesT.plot(ts, point[:, 2], 'b')
        axesT.legend(['x', 'y', 'z'])
        axesT.set_xlabel('Time (s)')
        axesT.set_ylabel('Coords (m)')
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axesT'] = axesT
        kwargs['axesR'] = axesR
        kwargs['minTime'] = minTime
        kwargs['maxTime'] = maxTime
        callback(**kwargs)

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
            X = marker_pose[:, 0]
            Y = marker_pose[:, 1]
            Z = marker_pose[:, 2]
            ax.scatter3D(X, Y, Z, label=name)

            # Create cubic bounding box to simulate equal aspect ratio
            max_range = np.array([X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]).max()
            Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (X.max() + X.min())
            Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (Y.max() + Y.min())
            Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (Z.max() + Z.min())
            # Comment or uncomment following both lines to test the fake bounding box:
            for xb, yb, zb in zip(Xb, Yb, Zb):
                ax.plot([xb], [yb], [zb], 'w')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = ax  # TODO: make this handling consistent across the library
        callback(**kwargs)
    # return ax # ax is also available to the calling function inside **kwargs
