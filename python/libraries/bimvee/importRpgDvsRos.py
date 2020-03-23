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
importRpgDvsRos function first uses importRosbag  function, 
which is a completely standard function to import a rosbag file,
resulting in a dictionary of conns (connections contained in the file), 
each containing 'msgs', a list of messages. 
This does not attempt interpretation of the messages. 
importRpgDvsRos then takes any connection which is of recognised types, and
uses only these messages to output an importedDict, in the format defined for importAe.

In particular, this supports the dvs_msgs/EventArray messages defined at:
http://rpg.ifi.uzh.ch/davis_data.html

It also supports some standard ros types:
    sensor_msgs/Image, 
    sensor_msgs/CameraInfo, (calibration)
    sensor_msgs/Imu 
    geometry_msgs/PoseStamped 
    
Return nested dicts of the form:
{
    info
    data
        0
            dvs
                "pol": numpy array of bool
                "x": numpy array of uint16
                "y": numpy array of uint16
                "ts": numpy array of float - seconds (basic format is int with unit increments of 80 ns) 
            frame ...
            imu ...
            etc ...
            
The function requires a template input to tell it which connections to map
to which channels and datatypes in the resulting dict. 
Here follow 2 example templates:
    
template = {
    'left': {
        'dvs': '/davis/left/events',
    }, 'right': {
        'dvs': '/davis/right/events',
        }
    }    
    
template = {
    'ch0': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        'pose6q': '/optitrack/davis',
        'cam': '/dvs/camera_info',
        'imu': '/dvs/imu'
        }
    }    

Any connections which are not named in the template are not imported but simply listed.
To just inspect the connections, pass in an empty template (or just don't pass in the template parameter)
Use this to inspect a new .bag file before defining the import template.
"""

#%%

from struct import unpack
from struct import error as structError
from tqdm import tqdm
import numpy as np
import string

# Local imports
if __package__ is None or __package__ == '':
    from timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts, unwrapTimestamps
else:
    from .timestamps import zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts, unwrapTimestamps

def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value

def unpackHeader(headerLen, headerBytes):
    fields = {}
    ptr = 0
    while ptr < headerLen:
        fieldLen = unpack('=l', headerBytes[ptr:ptr+4])[0]
        ptr += 4
        #print(fieldLen)
        field = headerBytes[ptr:ptr+fieldLen]
        ptr += fieldLen
        #print(field)
        fieldSplit = field.find(b'\x3d')
        fieldName = field[:fieldSplit].decode("utf-8")
        fieldValue = field[fieldSplit+1:]
        fields[fieldName] = fieldValue
    return fields

def readFile(filePathOrName):
    print('Attempting to import ' + filePathOrName + ' as a rosbag 2.0 file.')
    with open(filePathOrName, 'rb') as file:
        # File format string
        fileFormatString = file.readline().decode("utf-8")
        print('ROSBAG file format: ' + fileFormatString)
        if fileFormatString != '#ROSBAG V2.0\n':
            print('This file format might not be supported')
        eof = False
        conns = []
        chunks = []
        while not eof:
            # Read a record header
            try:
                headerLen = unpack('=l', file.read(4))[0]
            except structError:
                if len(file.read(1)) == 0: # Distinguish EOF from other struct errors 
                   # a struct error could also occur if the data is downloaded by one os and read by another.
                   eof = True
                   continue
            # unpack the header into fields 
            headerBytes = file.read(headerLen)
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', file.read(4))[0]
            data = file.read(dataLen)
            # The op code tells us what to do with the record
            op = unpack('=b', fields['op'])[0]
            fields['op'] = op
            if op == 2:
                # It's a message
                # AFAIK these are not found unpacked in the file
                #fields['data'] = data 
                #msgs.append(fields)
                pass
            elif op == 3:
                # It's a bag header - use this to do progress bar for the read
                chunkCount = unpack('=l', fields['chunk_count'])[0]
                pbar = tqdm(total=chunkCount, position=0, leave=True)
            elif op == 4:
                # It's an index - this is used to index the previous chunk
                conn = unpack('=l', fields['conn'])[0]
                count = unpack('=l', fields['count'])[0]
                for idx in range(count):
                    time, offset = unpack('=ql', data[idx*12:idx*12+12])
                    chunks[-1]['ids'].append((conn, time, offset))
            elif op == 5:
                # It's a chunk
                fields['data'] = data
                fields['ids'] = []
                chunks.append(fields)
                pbar.update(len(chunks))
            elif op == 6:
                # It's a chunk-info - seems to be redundant
                pass
            elif op == 7:
                # It's a conn
                # interpret data as a string containing the connection header
                connFields = unpackHeader(dataLen, data)
                connFields.update(fields) # to do: consider the correct handling of topic, which may be present in both
                connFields['conn'] = unpack('=l', connFields['conn'])[0]
                connFields['topic'] = connFields['topic'].decode("utf-8")
                conns.append(connFields)
    return conns, chunks

def unpackRosUint32(data, ptr):
    return unpack('=L', data[ptr:ptr+4])[0], ptr+4

def unpackRosString(data, ptr):
    stringLen = unpack('=L', data[ptr:ptr+4])[0]
    ptr += 4
    outStr = data[ptr:ptr+stringLen].decode('utf-8')
    ptr += stringLen
    return outStr, ptr

#%% Break chunks into msgs

def breakChunksIntoMsgs(chunks):
    msgs = [] 
    print('Breaking chunks into msgs ...')           
    for chunk in tqdm(chunks, position=0, leave=True):
        for idx in chunk['ids']:
            ptr = idx[2]
            headerLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            # unpack the header into fields 
            headerBytes = chunk['data'][ptr:ptr+headerLen]
            ptr += headerLen
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            fields['data'] = chunk['data'][ptr:ptr+dataLen]
            fields['conn'] = unpack('=l', fields['conn'])[0]
            msgs.append(fields)
    return msgs

def importRosbag(**kwargs):
    filePathOrName = kwargs.get('filePathOrName')
    print('Importing file: ', filePathOrName) 
    conns, chunks = readFile(filePathOrName)
    msgs = breakChunksIntoMsgs(chunks)
    # Restructure conns as a dictionary keyed by conn number
    connDict = {}
    for conn in conns:
        connDict[conn['conn']] = conn
        conn['msgs'] = []
    for msg in msgs:     
        connDict[msg['conn']]['msgs'].append(msg)
    # Now rekey the dictionary by topic
    topicDict = {}
    for conn in connDict:
        topicDict[connDict[conn]['topic']] = connDict[conn]
    return topicDict

def interpretMsgsAsDvs(msgs, **kwargs):
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    xAll = np.zeros((sizeOfArray), dtype=np.uint16)
    yAll = np.zeros((sizeOfArray), dtype=np.uint16)
    polAll = np.zeros((sizeOfArray), dtype=np.bool)
    for msg in tqdm(msgs, position=0, leave=True):
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        #timeS, timeNs = unpack('=LL', data[4:12])
        frame_id, ptr = unpackRosString(data, 12)
        ptr += 4 #height = unpack('=L', data[ptr:ptr+4])[0]
        ptr += 4 #width = unpack('=L', data[ptr:ptr+4])[0]
        numEventsInMsg, ptr = unpackRosUint32(data, ptr)
        while sizeOfArray < numEvents + numEventsInMsg:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            xAll = np.append(xAll, np.zeros((sizeOfArray), dtype=np.uint16))
            yAll = np.append(yAll, np.zeros((sizeOfArray), dtype=np.uint16))
            polAll = np.append(polAll, np.zeros((sizeOfArray), dtype=np.bool))
            sizeOfArray *= 2
        for idx in range(numEventsInMsg):
            idxAll = idx + numEvents
            x, y, ts, tns, pol = unpack('=HHLL?', data[ptr + idx*13 : ptr + (idx + 1)*13])
            tsFloat = np.float64(ts)+np.float64(tns)*0.000000001 # It's possible that this will kilL the precision
            xAll[idxAll] = x
            yAll[idxAll] = y
            tsAll[idxAll] = tsFloat
            polAll[idxAll] = pol
        numEvents += numEventsInMsg
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    xAll = xAll[:numEvents]
    yAll = yAll[:numEvents]
    polAll = polAll[:numEvents]
    outDict = {
        'x': xAll,
        'y': yAll,
        'ts': tsAll,
        'pol': polAll}
    return outDict

def interpretMsgsAsFrame(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    the result is a ts plus a 2d array of samples ()
    '''
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    framesAll = []
    for msg in tqdm(msgs, position=0, leave=True):
        if sizeOfArray < numEvents + 1:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            sizeOfArray *= 2            
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        if kwargs.get('useRosMsgTimestamps', False):
            timeS, timeNs = unpack('=LL', msg['time'])
        else:
            timeS, timeNs = unpack('=LL', data[4:12])
        tsAll[numEvents] = np.float64(timeS)+np.float64(timeNs)*0.000000001
        frame_id, ptr = unpackRosString(data, 12)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr)
        fmtString, ptr = unpackRosString(data, ptr)
        isBigendian = unpack('=B', data[ptr:ptr+1])[0]
        if isBigendian:
            print('data is bigendian, but it doesn''t matter')            
        ptr += 1
        ptr += 4 #step = unpack('=L', data[ptr:ptr+4])[0]
        # I don't know why, but there is an ip address in the next 4 bytes
        ptr += 4
        if fmtString in ['mono8', '8UC1']:
            frameData = np.frombuffer(data[ptr:ptr+height*width],np.uint8)
        elif fmtString == '32FC1':
            frameData = np.frombuffer(data[ptr:ptr+height*width*4],np.float32)
        else:
            print('image format not supported:' + fmtString)
            return None
        frameData = frameData.reshape(height, width)
        framesAll.append(frameData)
        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    outDict = {
        'ts': tsAll,
        'frames': framesAll}
    return outDict

def interpretMsgsAsPose6q(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    the result is a ts plus a 7-column np array of np.float64,
    where the cols are x, y, z, q-x, q-y, q-z, q-w (i.e. quaternion orientation)
    '''
    if kwargs.get('poseAsTransform', False):
        return interpretMsgsAsPose6qTransform(msgs, **kwargs)
    #if 'Stamped' not in kwargs.get('messageType', 'Stamped'):
    #    return interpretMsgsAsPose6qAlt(msgs, **kwargs)
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    poseAll = np.zeros((sizeOfArray, 7), dtype=np.float64)
    for msg in tqdm(msgs, position=0, leave=True):
        if sizeOfArray < numEvents + 1:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            poseAll = np.concatenate((poseAll, np.zeros((sizeOfArray, 7), dtype=np.float64)))
            sizeOfArray *= 2
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        timeS, timeNs = unpack('=LL', data[4:12])
        tsAll[numEvents] = np.float64(timeS)+np.float64(timeNs)*0.000000001 
        frame_id, ptr = unpackRosString(data, 12)
        poseAll[numEvents, :] = np.frombuffer(data[ptr:ptr+56], dtype=np.float64)
        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    poseAll = poseAll[:numEvents]
    point = poseAll[:, 0:3]
    rotation = poseAll[:, [6, 3, 4, 5]] # Switch quaternion form from xyzw to wxyz
    outDict = {
        'ts': tsAll,
        'point': point,
        'rotation': rotation}
    return outDict

def interpretMsgsAsPose6qTransform(msgs, **kwargs):
    '''
    Hmm - it's a bit hack how this is stuck in here at the moment, 
    but whereas rpg used PoseStamped message type, realsense use Transform:
    ros message is defined here:
        http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html
    the result is a 7-column np array of np.float64,
    where the cols are x, y, z, q-x, q-y, q-z, q-w (i.e. quaternion orientation)
    ts must therefore come from the message header; does that mean there is
    only one pose per message?
    '''
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    poseAll = np.zeros((sizeOfArray, 7), dtype=np.float64)
    for msg in tqdm(msgs, position=0, leave=True):
        if sizeOfArray < numEvents + 1:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            poseAll = np.concatenate((poseAll, np.zeros((sizeOfArray, 7), dtype=np.float64)))
            sizeOfArray *= 2
        # Note - ignoring kwargs['useRosMsgTimestamps'] as there is no choice
        timeS, timeNs = unpack('=LL', msg['time'])
        tsAll[numEvents] = np.float64(timeS)+np.float64(timeNs)*0.000000001
        poseAll[numEvents, :] = np.frombuffer(msg['data'], dtype=np.float64)
        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    tsUnwrapped = unwrapTimestamps(tsAll) # here we could pass in wrapTime=2**62, but actually it handles this internally
    poseAll = poseAll[:numEvents]
    point = poseAll[:, 0:3]
    rotation = poseAll[:, [6, 3, 4, 5]] # Switch quaternion form from xyzw to wxyz
    outDict = {
        'ts': tsUnwrapped,
        'point': point,
        'rotation': rotation}
    return outDict


def interpretMsgsAsImu(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    the result is are np arrays of float64 for:
        rotQ (4 cols, quaternion)
        angV (3 cols)
        acc (3 cols)
        mag (3 cols)
        temp (1 cols) - but I'll probably ignore this to start with
    '''
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    rotQAll = np.zeros((sizeOfArray, 4), dtype=np.float64)
    angVAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    accAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    magAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    #tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    for msg in tqdm(msgs, position=0, leave=True):
        if sizeOfArray < numEvents + 1:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            rotQAll = np.concatenate((rotQAll, np.zeros((sizeOfArray, 4), dtype=np.float64)))
            angVAll = np.concatenate((angVAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            accAll = np.concatenate((accAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            magAll = np.concatenate((magAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            sizeOfArray *= 2
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        timeS, timeNs = unpack('=LL', data[4:12])
        tsAll[numEvents] = np.float64(timeS)+np.float64(timeNs)*0.000000001 
        frame_id, ptr = unpackRosString(data, 12)
        rotQAll[numEvents, :] = np.frombuffer(data[ptr:ptr+32], np.float64)
        ptr += 32
        ptr += 72 # Skip the covariance matrix
        angVAll[numEvents, :] = np.frombuffer(data[ptr:ptr+24], np.float64)
        ptr += 24
        ptr += 72 # Skip the covariance matrix
        accAll[numEvents, :] = np.frombuffer(data[ptr:ptr+24], np.float64)
        #ptr += 24
        #ptr += 72 # Skip the covariance matrix
        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    rotQAll = rotQAll[:numEvents]
    angVAll = angVAll[:numEvents]
    accAll = accAll[:numEvents]
    magAll = magAll[:numEvents]
    outDict = {
        'ts': tsAll,
        'rotQ': rotQAll,
        'angV': angVAll,
        'acc': accAll,
        'mag': magAll
        }
    return outDict

def interpretMsgsAsCam(msgs, **kwargs):
    '''
    camera info - i.e. results of calibration
    ros message is defined here:
        http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    We assume that there will only be one camera_info msg per channel,
    so the resulting dict is populated by the following fields:
        std_msgs/Header header
        uint32 height
        uint32 width
        string distortion_model - actually not storing this for now, just checking that it's "plumb_bob"
        <following as numpy matrices of the appropriate dimensions>
        float64[] D (distortion params)
        float64[9] K (Intrinsic camera matrix)
        float64[9] R (rectification matrix - only for stereo setup)
        float64[12] P (projection matrix)
        <ignoring the following for now:>
        uint32 binning_x
        uint32 binning_y
        sensor_msgs/RegionOfInterest roi
    '''
    outDict = {}
    data = msgs[0]['data'] # There is one calibration msg per frame. Just use the first one
    #seq = unpack('=L', data[0:4])[0]
    #timeS, timeNs = unpack('=LL', data[4:12])
    frame_id, ptr = unpackRosString(data, 12)
    outDict['height'], ptr = unpackRosUint32(data, ptr)
    outDict['width'], ptr = unpackRosUint32(data, ptr)
    outDict['distortionModel'], ptr = unpackRosString(data, ptr)
    ptrAfterDistortionModel = ptr
    if outDict['distortionModel'] == 'plumb_bob':
        try:
            ptr +=4 # There's an IP address in there, not sure why
            outDict['D'] = np.frombuffer(data[ptr:ptr+40], dtype=np.float64)
            ptr += 40
            outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
        except ValueError:
            # 2020_02 Sim: When we run ESIM it is not outputting the D matrix, not sure why 
            # TODO: Either we modify the way we use ESIM, or else, if there's a reason for the absence of D, 
            # allow for that here in a less hacky way.
            ptr = ptrAfterDistortionModel + 4 # Allow for IP address
            outDict['D'] = np.zeros((5), dtype=np.float64)
            #ptr += 40
            outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    elif outDict['distortionModel'] == 'Kannala Brandt4':
        ptr +=4 # There's an IP address in there, not sure why
        outDict['D'] = np.frombuffer(data[ptr:ptr+32], dtype=np.float64)
        ptr += 40
        outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    else:
        #print('Distortion model not supported:' + outDict['distortionModel'])
        #return None
        # simulated data doesn't have a distortion model name, so try again like this
        frame_id, ptr = unpackRosString(data, 12)
        outDict['height'], ptr = unpackRosUint32(data, ptr)
        outDict['width'], ptr = unpackRosUint32(data, ptr)
        outDict['D'] = np.frombuffer(data[ptr:ptr+40], dtype=np.float64)
        ptr += 40
        outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    #ptr += 96
    # Ignore binning and ROI
    return outDict

def utiliseCameraInfoWithinChannel(channelDict):
    if 'cam' in channelDict and 'height' in channelDict['cam']:
        if 'dvs' in channelDict and 'dimY' not in channelDict['dvs']:
            channelDict['dvs']['dimX'] = channelDict['cam']['width']
            channelDict['dvs']['dimY'] = channelDict['cam']['height']
            
def importRpgDvsRos(**kwargs):
    topics = importRosbag(**kwargs)
    template = kwargs.get('template', {})
    # The template becomes the data branch of the importedDict
    outDict = {
        'info': kwargs,
        'data': {}
            }            
    for channelKey in template:
        print('Importing for channel : ', channelKey)
        channelKeyStripped = str(channelKey).translate(str.maketrans('', '', string.punctuation))
        outDict['data'][channelKeyStripped] = {}
        for dataType in template[channelKey]:
            topic = template[channelKey][dataType]
            topic = topics.pop(topic)
            msgs = topic['msgs']
            print('Importing for dataType "' + dataType + '"; there are ' + str(len(msgs)) + ' messages')
            if dataType == 'dvs':
                # Interpret these messages as EventArray
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsDvs(msgs, **kwargs)
            elif dataType == 'pose6q':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsPose6q(msgs, **kwargs)
            elif dataType == 'frame':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsFrame(msgs, **kwargs)
            elif dataType == 'cam':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsCam(msgs, **kwargs)
            elif dataType == 'imu':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsImu(msgs, **kwargs)
            else:
                print('dataType "', dataType, '" not recognised.')
        if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
            # Optional: start the timestamps at zero for the first event
            # This is done collectively for all the concurrent imports
            zeroTimestampsForAChannel(outDict['data'][channelKeyStripped])
    # jointly rezero for all channels
    if kwargs.get('zeroTimestamps', True):
        # Optional: start the timestamps at zero for the first event
        # This is done collectively for all the concurrent imports
        rezeroTimestampsForImportedDicts(outDict)
    # report the remaining topics
    remainingTopics = topics.keys()
    if remainingTopics:
        print('The following topics are present in the file but were not imported: ')
        for topic in remainingTopics:
            print(topic)
    # if cam and dvs exist in the same channel, apply height and width to dimY/X
    for channelKey in outDict['data'].keys():
        utiliseCameraInfoWithinChannel(outDict['data'][channelKey])
    return outDict
    '''   

    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    chAll = np.zeros((sizeOfArray), dtype=np.uint8)
    xAll = np.zeros((sizeOfArray), dtype=np.uint16)
    yAll = np.zeros((sizeOfArray), dtype=np.uint16)
    polAll = np.zeros((sizeOfArray), dtype=np.bool)
    '''
