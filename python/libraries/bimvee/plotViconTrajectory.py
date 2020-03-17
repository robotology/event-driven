# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Suman Ghosh
        Simeon Bamford
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Library for plotting vicon data parsed using importVicon.py

"""

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

"""
Plot the 3D trajectory of a body or marker for the entire duration of recording

To select which bodies to plot using bodyID: 
 - look at the bodyIDs parsed using importVicon
 - pass a list of strings present in the bodyID of choice, through the parameter include
 - pass a list of strings that should be absent from the bodyID of choice, through the parameter exclude
"""

def plot_trajectories(viconDataDict, bodyIds, include, exclude, **kwargs):
    ax = kwargs.get('ax')
    if ax is None:
        ax = plt.axes(projection='3d')
        kwargs['ax'] = ax
    for name in bodyIds:
        select_body = all([(inc in name) for inc in include]) and all([not (exc in name) for exc in exclude])
        if select_body:  # modify this line to plot whichever markers you want
            marker_pose = viconDataDict['data'][name]['pose6q']['point']
            ax.scatter3D(marker_pose[:, 0], marker_pose[:, 1], marker_pose[:, 2], label=name)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.legend()
    callback = kwargs.get('callback')
    if callback is not None:
        kwargs['axes'] = ax # TODO: make this handling consistent across the library
        callback(**kwargs)
    #return ax # ax is also available to the calling function inside **kwargs

