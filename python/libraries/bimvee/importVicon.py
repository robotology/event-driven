# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Author: Suman Ghosh
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Input:
vicon data log recorded through yarp

Output:
outDict = { 'info' : <filepath>, 'data' : { <bodyID> : { 'ts' : <1D array of timestamps>,
            'pose7d' : <2D array where each row represents 7d pose of a body at a time instant> } } }

A bodyID is the name assigned by vicon to a marker (labeled / unlableled) or rigid body
The 7d pose is in the form [x, y, z, r_x, r_y, r_z, r_w] with the orientation as a quaternion

"""

import re
import numpy as np

def importVicon(filePathOrName):
    pattern = re.compile('(\d+) (\d+\.\d+) \((.*)\)')
    # yarpBottleTimes = []
    outDict = {'info': filePathOrName, 'data': {}}
    bodyids = []

    with open(filePathOrName, 'r') as file:
        print('Found file to read')
        line = file.readline()
        while line:
            found = pattern.search(line.strip())
            # yarpBottleTimes.append(float(found.group(2)))
            viconData = found.group(3)
            bodies = viconData.split(') (')
            for body in bodies:
                elements = body.split(" ")
                bodyID = elements[1].strip('\"')
                ts = elements[2]
                pose = elements[3:]
                try:
                    poseDict = outDict['data'][bodyID]['pose']
                except KeyError:
                    print('KeyError exception.. Creating new key', bodyID)
                    bodyids.append(bodyID)
                    outDict['data'][bodyID] = {'pose': {'ts': [], 'pose7d': []}}
                    poseDict = outDict['data'][bodyID]['pose']
                poseDict['ts'].append(ts)
                poseDict['pose7d'].append(pose)
            line = file.readline()

    # converting lists of strings to numpy arrays of float64
    for id in bodyids:
        outDict['data'][id]['pose']['ts'] = np.array(outDict['data'][id]['pose']['ts'], dtype=np.float64)
        outDict['data'][id]['pose']['pose7d'] = np.array(outDict['data'][id]['pose']['pose7d'], dtype=np.float64)

    return outDict, bodyids
