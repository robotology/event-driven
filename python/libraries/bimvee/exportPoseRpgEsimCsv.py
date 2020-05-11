# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Simeon Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
A function which exports a pose6q dataType dict as a csv for use by Rpg's eSim
simulator (https://github.com/uzh-rpg/rpg_esim)

Note:
    a) conversion of ts to ns
    b) switch of quaternion to qx, qy, qz, qw format
"""


def exportPoseRpgEsimCsv(poseDict, filePathAndName='poses.csv'):
    ts = poseDict['ts']
    point = poseDict['point']
    rotation = poseDict['rotation']
    with open(filePathAndName, 'w') as file:
        file.write('# timestamp, x, y, z, qx, qy, qz, qw\n')
        for idx in range(ts.shape[0]):
            file.write('%f, %f, %f, %f, %f, %f, %f, %f\n' % (
                    ts[idx] * 1000000000,                                                       
                    point[idx, 0],                                                       
                    point[idx, 1],                                                       
                    point[idx, 2],                                                       
                    rotation[idx, 1],                                                       
                    rotation[idx, 2],                                                       
                    rotation[idx, 3],                                                       
                    rotation[idx, 0],                                                       
                    ))
