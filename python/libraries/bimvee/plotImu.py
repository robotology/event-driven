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
plotImu takes 'inDict' - a dictionary containing imported IMU data 
(or a higher level container, in which attempts to descend and call itself) 
as created by importAe, and plots against time the various dimensions of the 
IMU samples contained. 
"""
import matplotlib.pyplot as plt

def plotImu(inDict, **kwargs):
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plotImu(inDictInst, **kwargs)
        return
    if 'info' in inDict:
        fileName = inDict['info'].get('filePathOrName', '')
        print('plotImu was called for file ' + fileName)
        if not inDict['data']:
            print('The import contains no data.')
            return
        for channelName in inDict['data']:
            channelData = inDict['data'][channelName]
            if 'imu' in channelData and len(channelData['imu']['ts']) > 0:
                kwargs['title'] = ' '.join([fileName, str(channelName)])
                plotImu(channelData['imu'], **kwargs)
            else:
                print('Channel ' + channelName + ' skipped because it contains no polarity data')
        return
    fig, allAxes = plt.subplots(4, 1)
    fig.suptitle(kwargs.get('title', ''))
    axesAcc = allAxes[0]
    axesAcc.plot(inDict['ts'], inDict['acc'])
    axesAcc.set_title('Acceleration (m/s)')
    axesAcc.legend(['x', 'y', 'z'])

    axesAngV = allAxes[1]
    axesAngV.plot(inDict['ts'], inDict['angV'])
    axesAngV.set_title('Angular velocity (rad/s)')
    axesAngV.legend(['x', 'y', 'z'])

    if 'temp' in inDict: 
        axesTemp = allAxes[2]
        axesTemp.plot(inDict['ts'], inDict['temp'])
        axesTemp.set_title('Temp (K)')

    axesMag = allAxes[3]
    axesMag.plot(inDict['ts'], inDict['mag'])
    axesMag.set_title('Mag (?)')
    axesMag.legend(['x', 'y', 'z'])




