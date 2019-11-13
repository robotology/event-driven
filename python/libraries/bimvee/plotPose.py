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
plotPose takes 'inDict' - a dictionary containing imported pose data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and plots against time the various dimensions of the 
imu samples contained. 
"""
import matplotlib.pyplot as plt

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
            else:
                print('Channel ' + channelName + ' skipped because it contains no pose data')
        return    
    
    if kwargs.get('zeroT', False):
        inDict['pose'][:, 0] = inDict['pose'][:, 0] - inDict['pose'][0, 0]
        inDict['pose'][:, 1] = inDict['pose'][:, 1] - inDict['pose'][0, 1]
        inDict['pose'][:, 2] = inDict['pose'][:, 2] - inDict['pose'][0, 2]
    fig, allAxes = plt.subplots(2, 1)
    axesT = allAxes[0]
    axesT.plot(inDict['ts'], inDict['pose'][:, :3])
    axesT.legend(['x', 'y', 'z'])
    axesR = allAxes[1]
    axesR.plot(inDict['ts'], inDict['pose'][:, 3:])
    axesR.legend(['r_x', 'r_y', 'r_z', 'r_w'])
