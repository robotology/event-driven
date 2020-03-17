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
Takes a dict as imported by importAe, and for each channel, 
tries to run the appropriate general visualisation function
"""

# local imports
if __package__ is None or __package__ == '':
    from plotDvsContrast import plotDvsContrast
    from plotEventRate import plotEventRate
    from plotFrame import plotFrame
    from plotPose import plotPose
    from plotImu import plotImu
else:
    from .plotDvsContrast import plotDvsContrast
    from .plotEventRate import plotEventRate
    from .plotFrame import plotFrame
    from .plotPose import plotPose
    from .plotImu import plotImu

def plot(inDict, **kwargs):
    if isinstance(inDict, list):
        for inDictInst in inDict:
            plot(inDictInst, **kwargs)
        return
    filePathOrName = inDict['info'].get('filePathOrName', '')
    for channelName in inDict['data'].keys():
        channel = inDict['data'][channelName]
        title = filePathOrName + '-' + channelName
        kwargs['title'] = title
        if 'dvs' in channel:
            plotDvsContrast(channel['dvs'], **kwargs)
            plotEventRate(channel['dvs'], **kwargs)
        if 'frame' in channel:
            plotFrame(channel['frame'], **kwargs)
        if 'imu' in channel:
            plotImu(channel['imu'], **kwargs)
        if 'pose6q' in channel:
            plotPose(channel['pose6q'], **kwargs)
        if 'point3' in channel:
            plotPose(channel['point3'], **kwargs)
