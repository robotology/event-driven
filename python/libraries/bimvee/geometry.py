# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Author: Sim Bamford

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
Low-level gemeotry functions, 
for example, for manipulating poses, orientations, projections etc
"""

import numpy as np

# local imports
if __package__ is None or __package__ == '':
    from timestamps import sortDataTypeDictByTime
else:
    from .timestamps import sortDataTypeDictByTime

# Can accept an existing matrix, which should be min 3x3; 
#if it creates a matrix it makes it 4x4
def quat2RotM(quat, M=None):
    if M is None: 
        M = np.zeros((4, 4))
        M[3, 3] = 1
    elif M.shape[0] == 3:
        M[:, 3] = 0
        M[3, :] = 0
        M[3, 3] = 1
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    M[0, 0] = 1 - 2*y**2 - 2*z**2
    M[0, 1] = 2*x*y - 2*z*w
    M[0, 2] = 2*x*z + 2*y*w
    M[1, 0] = 2*x*y + 2*z*w
    M[1, 1] = 1 - 2*x**2 - 2*z**2
    M[1, 2] = 2*y*z - 2*x*w
    M[2, 0] = 2*x*z - 2*y*w
    M[2, 1] = 2*y*z + 2*x*w
    M[2, 2] = 1 - 2*x**2 - 2*y**2
    return M

def rotateUnitVectors(rotM, unitLength=1.0):
    xVec = np.expand_dims(np.array([unitLength, 0, 0, 1]), axis=1)
    yVec = np.expand_dims(np.array([0, unitLength, 0, 1]), axis=1)
    zVec = np.expand_dims(np.array([0, 0, unitLength, 1]), axis=1)
    allVecs = np.concatenate((xVec, yVec, zVec), axis=1)
    return rotM.dot(allVecs)

# Spherical linear interpolation, adapted from https://en.wikipedia.org/wiki/Slerp
DOT_THRESHOLD = 0.9995
def slerp(q1, q2, time_relative):
    dot = np.sum(q1 * q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > DOT_THRESHOLD:
        result = q1 + time_relative * (q2 - q1)
        return (result.T / np.linalg.norm(result)).T
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * time_relative
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q1) + (s1 * q2)

''' 
expects pose dict in the form: {'ts': 1d np.array of np.float64 timestamps,
                                'point': 2d array np.float64 of positions [x, y, z], 
                                'rotation': 2d array np.float64 of quaternions [rw, rx, ry, rz]
                                (i.e. 6dof with rotation as quaternion)}
Two modes of operation:
If time is not None, then returns the interpolated pose at that time -
    returns (point, rotation) tuple, being np.array 1d x 3 and 4 respectively, np.float64,    which is interpolated pose;
Else if maxPeriod is not None, then it returns the entire pose dict,
    but with additional points necessary to ensure that time between samples 
    never exceeds maxPeriod
'''
def pose6qInterp(poseDict, time=None, maxPeriod=None):
    ts = poseDict['ts']
    points = poseDict['point']
    rotations = poseDict['rotation']
    if time is not None:
        idxPre = np.searchsorted(ts, time, side='right') - 1
        timePre = ts[idxPre]
        if timePre == time:
            # In this edge-case of desired time == timestamp, there is no need 
            # to interpolate 
            return (points[idxPre, :], rotations[idxPre, :])
        if idxPre < 0:
            return (points[0, :], rotations[0, :])
        if idxPre >= len(poseDict['ts']):
            return (points[-1, :], rotations[-1, :])
        timePost = ts[idxPre + 1]
        rotPre = rotations[idxPre, :]
        rotPost = rotations[idxPre + 1, :]
        timeRel = (time - timePre) / (timePost - timePre)
        rotOut = slerp(rotPre, rotPost, timeRel)
        pointPre = points[idxPre, :] 
        pointPost = points[idxPre + 1, :]
        pointOut = pointPre * (1-timeRel) + pointPost * timeRel
        return (pointOut, rotOut)
    elif maxPeriod is not None:
        firstTime = ts[0]
        lastTime = ts[-1]
        proposedAdditionalTimes = np.arange(firstTime, lastTime, maxPeriod)
        prevIds = np.searchsorted(ts, proposedAdditionalTimes, side='right') - 1
        distPre = proposedAdditionalTimes - ts[prevIds]
        distPost = ts[prevIds + 1] - proposedAdditionalTimes
        dist = distPre + distPost
        keepAdditional = dist > maxPeriod
        additionalTimes = proposedAdditionalTimes[keepAdditional]
        additionalPoses = [pose6qInterp(poseDict, time=additionalTime) 
                            for additionalTime in additionalTimes] 
        additionalPoints, additionalRotations = zip(*additionalPoses)
        ts = np.concatenate((ts, additionalTimes))
        points = np.concatenate((points, np.array(additionalPoints)))
        rotations = np.concatenate((rotations, np.array(additionalRotations)))
        poseDict['ts'] = ts
        poseDict['point'] = points
        poseDict['rotation'] = rotations
        poseDict = sortDataTypeDictByTime(poseDict)
        return poseDict

def quaternionProduct(q1, q2):
    return np.array([ 
    q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
    q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
    q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] ])
    ''' alternative definition - less efficient
    w1 = q1[0]
    v1 = q1[1:4]
    w2 = q2[0]
    v2 = q2[1:4]
    wOut = w1*w2 - np.dot(v1.T, v2)
    vOut = w1*v2 + w2*v1 + np.cross(v1, v2)
    qOut = np.zeros_like(q1)
    qOut[0] = wOut
    qOut[1:4] = vOut
    return qOut
    '''

# Legacy name for Quaternion multiplication
def combineTwoQuaternions(q1, q2):
    return quaternionProduct(q1, q2)

def quaternionConjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternionInverse(q):
    return quaternionConjugate(q) / np.sum(q ** 2)

def angleBetweenTwoQuaternions(q1, q2):
    # returns minimal angle in radians between two unit quaternions, following:
    # https://www.researchgate.net/post/How_do_I_calculate_the_smallest_angle_between_two_quaternions    
    qP = quaternionProduct(q1, quaternionInverse(q2))
    normV = np.linalg.norm(qP[1:])
    return 2 * np.arcsin(normV)

'''
Expects 
    - poseDict in bimvee form {'ts', 'point', 'rotation' as above}
    - translation as np array of x,y,z
    - rotation as a np array of w,x,y,z (quaternion)
If either translation or rotation are passed, these are applied to all poses 
in the poseDict.
The rotations are defined wrt local axes, unlike the translations.
Returns a copy of the poseDict, rotated
'''    
def transformPoses(poseDict, translation=None, rotation=None):
    # Create a copy of the input array - use the same contents
    outDict = {}
    for key in poseDict.keys():
        outDict[key] = poseDict[key]
    if translation is not None:
        outDict['point'] = poseDict['point'] + translation
    if rotation is not None:
        for idx in range(outDict['rotation'].shape[0]): # TODO: this could be matricised
            outDict['rotation'][idx, :] = \
                combineTwoQuaternions(outDict['rotation'][idx, :], rotation)
    return outDict
        
# adapted from https://stackoverflow.com/questions/50387606/python-draw-line-between-two-coordinates-in-a-matrix
def draw_line(mat, x0, y0, x1, y1):
    if (x0, y0) == (x1, y1):
        if x0 >= 0 and x0 < mat.shape[1] and y0 >= 0 and y0 < mat.shape[0]:
            mat[x0, y0] = 255
        return
    # Swap axes if Y slope is smaller than X slope
    transpose = abs(x1 - x0) < abs(y1 - y0)
    if transpose:
        mat = mat.T
        x0, y0, x1, y1 = y0, x0, y1, x1
    # Swap line direction to go left-to-right if necessary
    if x0 > x1:
        x0, y0, x1, y1 = x1, y1, x0, y0
    # Compute intermediate coordinates using line equation
    x = np.arange(x0, x1 + 1)
    y = np.round(((y1 - y0) / (x1 - x0)) * (x - x0) + y0).astype(x.dtype)
    # Write intermediate coordinates
    toKeep = np.logical_and(x >= 0, 
                            np.logical_and(x<mat.shape[1], 
                                           np.logical_and(y >= 0, y<mat.shape[0])))
    mat[y[toKeep], x[toKeep]] = 255

# The following adapted from https://github.com/christophhagen/averaging-quaternions
# Note that the signs of the output quaternion can be reversed, 
# since q and -q describe the same orientation.
# w is an optional weight array, which must have the same number of elements 
# as the number of quaternions
def averageOfQuaternions(allQ, w=None):
    # Number of quaternions to average
    M = allQ.shape[0]
    A = np.zeros((4,4), dtype = np.float64)
    if w is None:
        w = np.ones(M,)
    weightSum = 0
    for i in range(0, M):
        q = allQ[i, :]
        # multiply q with its transposed version q' and add A
        A = w[i] * np.outer(q,q) + A
        weightSum += w[i]
    # scale
    A = (1.0 / M) * A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return eigenVectors[:,0]
