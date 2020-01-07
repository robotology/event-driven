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
"""

import re
import numpy as np

def importVicon(filePathOrName):
    pattern = re.compile('(\d+) (\d+\.\d+) \((.*)\)')
    yarpBottleTimes = []
    outDict = {'info': filePathOrName, 'data': {}}

    with open(filePathOrName, 'r') as file:
        print('Found file to read')
        line = file.readline()
        while line:
            found = pattern.search(line.strip())
            yarpBottleTimes.append(float(found.group(2)))
            viconData = found.group(3)
            bodies = viconData.split(') (')
            for body in bodies:
                elements = body.split(" ")
                bodyID = elements[1].strip('\"')
                ts = float(elements[2])
                pose = [float(i) for i in elements[3:]]
                try:
                    poseDict = outDict['data'][bodyID]['pose']
                except KeyError:
                    print('KeyError exception.. Creating new key', bodyID)
                    outDict['data'][bodyID] = {'pose': {'ts': [], 'pose': []}}
                    poseDict = outDict['data'][bodyID]['pose']
                poseDict['ts'].append(ts)
                poseDict['pose'].append(pose)
            line = file.readline()

    return outDict, yarpBottleTimes
