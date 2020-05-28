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
import os, sys
import matplotlib.pyplot as plt
import seaborn as sns

sns.set(palette="colorblind")

# Color Palette for Color Blindness
zesty_palette     = ['#F5793A', '#A95AA1', '#85C0F9', '#0F2080']
retro_palette     = ['#601A4A', '#EE442F', '#63ACBE', '#F9F4EC']
corporate_palette = ['#8DB8AD', '#EBE7E0', '#C6D4E1', '#44749D']

#-----------------------------------------------------------------------------------------------------
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

#-----------------------------------------------------------------------------------------------------
def plotImuDistribution(imuDict, unitIMU='FPGA', fig_path=None, fig_name=None, fig_subtitle=None):
    """
    Plot the distribution of the IMU data in imuDict. If specified, save the
    generated figure as fig_name.png at the location defined by fig_path.

    Arguments:
        imuDict {dict} -- dictionary of IMU data (as formatted by bimvee)

    Keyword Arguments:
        unitIMU {str} -- either 'FPGA' or 'SI' (default: {'FPGA'})
        fig_path {string} -- save path for the generated figure (default: {None})
        fig_name {string} -- name of the generated figure (default: {None})
        fig_subtitle {string} -- figure sub-title (default: {None})
    """
    fig = plt.figure(figsize=(14.0, 10.0))
    if isinstance(fig_subtitle, str):
        fig.suptitle("IMU Samples Distribution\n" + fig_subtitle, fontsize=20, fontweight='bold')
    else:
        fig.suptitle("IMU Samples Distribution", fontsize=20, fontweight='bold')

    plt.subplot(3,2,1)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 0], bins=100, color=zesty_palette[0])
    plt.title("Accelerometer", fontsize=16, fontweight='bold')
    if unitIMU == 'FPGA':
        plt.xlabel('accX [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('accX [m/s]', fontsize=10, fontweight='bold')

    plt.subplot(3,2,2)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 3], bins=100, color=zesty_palette[0])
    plt.title("Gyroscope", fontsize=16, fontweight='bold')
    if unitIMU == 'FPGA':
        plt.xlabel('gyroX [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('gyroX [rad/s]', fontsize=10, fontweight='bold')

    plt.subplot(3,2,3)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 1], bins=100, color=zesty_palette[1])
    if unitIMU == 'FPGA':
        plt.xlabel('accY [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('accY [m/s]', fontsize=10, fontweight='bold')

    plt.subplot(3,2,4)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 4], bins=100, color=zesty_palette[1])
    if unitIMU == 'FPGA':
        plt.xlabel('gyroY [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('gyroY [rad/s]', fontsize=10, fontweight='bold')

    plt.subplot(3,2,5)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 2], bins=100, color=zesty_palette[3])
    if unitIMU == 'FPGA':
        plt.xlabel('accZ [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('accZ [m/s]', fontsize=10, fontweight='bold')

    plt.subplot(3,2,6)
    sns.distplot([v for v, s in zip(imuDict['value'], imuDict['sensor']) if s == 5], bins=100, color=zesty_palette[3])
    if unitIMU == 'FPGA':
        plt.xlabel('gyroZ [fpga]', fontsize=10, fontweight='bold')
    elif unitIMU == 'SI':
        plt.xlabel('gyroZ [rad/s]', fontsize=10, fontweight='bold')

    fig.tight_layout()
    if isinstance(fig_subtitle, str):
        fig.subplots_adjust(top=0.85)
    else:
        fig.subplots_adjust(top=0.9)

    if isinstance(fig_path, str) and isinstance(fig_name, str): 
        plt.savefig(os.path.join(fig_path, fig_name + '.png'), dpi=300, bbox_inches='tight')
        print("Saving " + fig_name + ".png")
        plt.close()
    else:
        plt.show()
