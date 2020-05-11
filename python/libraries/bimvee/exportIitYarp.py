# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
Code contributions from:
        Vadim Tikhanoff
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
exportIitYarp takes a data dict resulting from importAe.
Exports data.log and info.log for each channel contained.
Uses the "exportFilePath" parameter for the top-level folder,
creating it if it doesn't exist. This can be relative or absolute.
Defaults to the current directory.
Uses the "yarpOutputPort" parameter for the port defined in info.log.
Default is "/file" - then the following is appended: "/AE:o"
Where more than one channel is contained, the channel name is used in place of "AE".
If the "viewerApp" parameter is True (default = True), then a play.xml 
file is created at the top level directory, which is configured so that when
run from yarpmanager, the data which has been exported can be viewed.
"writeProtected" (default=True) protects any existing data in the target exportFilePath from being overwritten.
"writeMode" (default='w') can be set to append mode ('a') for exporting (large) files in batches.
"""

#%%

import os
import numpy as np
from tqdm import tqdm
import string

# TODO: this could come from header info in some cases
def getDims(dataTypeDict):
    if 'pol' in dataTypeDict: # dvs
        minX = np.min(dataTypeDict['x'])
        maxX = np.max(dataTypeDict['x'])
        minY = np.min(dataTypeDict['y'])
        maxY = np.max(dataTypeDict['y'])
    elif 'frames' in dataTypeDict:
        minX = 0
        maxX = (dataTypeDict['frames'][0].shape)[0]
        minY = 0
        maxY = (dataTypeDict['frames'][0].shape)[1]
    return minX, maxX, minY, maxY
    
def writeModule(xmlFile, name, **kwargs):
    xmlFile.write('<module>\n')
    xmlFile.write('    <name> ' + name + ' </name>\n')
    parameters = kwargs.get('parameters', '')
    xmlFile.write('    <parameters> ' + parameters + ' </parameters>\n')
    dependencies = kwargs.get('dependencies', '')
    if dependencies != '':
        xmlFile.write('    <dependencies>\n')
        xmlFile.write('        ' + dependencies + '\n')
        xmlFile.write('    </dependencies>\n')
    xmlFile.write('    <node> localhost </node>\n')
    xmlFile.write('</module>\n')
    xmlFile.write('\n')

def writeConnection(xmlFile, fromPort, toPort, **kwargs):
    xmlFile.write('<connection>\n')
    xmlFile.write('    <from> ' + fromPort + ' </from>\n')
    xmlFile.write('    <to> ' + toPort + ' </to>\n')
    protocol = kwargs.get('protocol', 'udp')
    xmlFile.write('<protocol> ' + protocol + ' </protocol>\n')
    xmlFile.write('</connection>\n')
    xmlFile.write('\n')

def exportIitYarpViewer(importedDict, **kwargs):
    exportFilePath = kwargs.get('exportFilePath', './')
    with open(os.path.join(exportFilePath, 'play.xml'), 'w') as xmlFile:
        xmlFile.write('<application>\n')
        xmlFile.write('<name>play </name>\n')
        xmlFile.write('\n')
        writeModule(xmlFile=xmlFile, name='yarpdataplayer')
        pathForPlayback = kwargs.get('pathForPlayback', exportFilePath)
        #dependencyStr = ''
        dependencyStr = '<port timeout="100.0" request="load ' + pathForPlayback + '" reply="[ok]">/yarpdataplayer/rpc:i</port>'
        dataTypesToExport = kwargs.get('dataTypes')
        for channelKey in importedDict['data']:
            for dataType in importedDict['data'][channelKey]:
                if dataTypesToExport is not None and dataType not in dataTypesToExport: continue
                channelKeyStripped = str(channelKey).translate(str.maketrans('', '', string.punctuation))
                channelAndDataType = channelKeyStripped + dataType

                if dataType == 'dvs':
                    minX, maxX, minY, maxY = getDims(importedDict['data'][channelKey][dataType])
                    height = maxY + 1
                    width = maxX + 1
                    paramsFramer = '--name /framer' + channelAndDataType + \
                                                   ' --height ' + str(height) + \
                                                   ' --width ' + str(width) + \
                                                   ' --displays "(/' + channelAndDataType + ' (AE ISO))"'
                    writeModule(xmlFile=xmlFile, name='vFramerLite', parameters=paramsFramer, dependencies=dependencyStr)
                    dependencyStr = ''                    
                    paramsViewer = '--name /viewer/' + channelAndDataType + \
                                    ' --w ' + str((width) * 2) + \
                                    ' --h ' + str((height) * 2) + ' --synch'
                    writeModule(xmlFile=xmlFile, name='yarpview', parameters=paramsViewer)
                    
                    fromPort = '/file/' + channelAndDataType + ':o'
                    toPort = '/framer' + channelAndDataType + '/' + channelAndDataType + '/AE:i'
                    writeConnection(xmlFile=xmlFile, fromPort=fromPort, toPort=toPort)
                    
                    fromPort = '/framer' + channelAndDataType + '/' + channelAndDataType + '/image:o'
                    toPort = '/viewer/' + channelAndDataType 
                    writeConnection(xmlFile=xmlFile, fromPort=fromPort, toPort=toPort)
                    
                if dataType == 'frame':
                    minX, maxX, minY, maxY = getDims(importedDict['data'][channelKey][dataType])
                    height = maxY + 1
                    width = maxX + 1
                    paramsViewer = '--name /viewer/' + channelAndDataType + \
                                    ' --w ' + str((width) * 2) + \
                                    ' --h ' + str((height) * 2) + ' --synch' + \
                                    ' --RefreshTime 33'
                    writeModule(xmlFile=xmlFile, name='yarpview', parameters=paramsViewer, dependencies=dependencyStr)
                    dependencyStr = ''                    

                    fromPort = '/file/' + channelAndDataType + ':o'
                    toPort = '/viewer/' + channelAndDataType 
                    writeConnection(xmlFile=xmlFile, fromPort=fromPort, toPort=toPort)

                if dataType == 'imu':
                    paramsFramer = '--name /framer' + channelAndDataType + \
                                                  ' --height 200' + \
                                                  ' --width 200' + \
                                                  ' --displays "(/' + channelAndDataType + ' (AE IMU))"'
                    writeModule(xmlFile=xmlFile, name='vFramerLite', parameters=paramsFramer, dependencies=dependencyStr)
                    dependencyStr = ''                    
                    paramsViewer = '--name /viewer/' + channelAndDataType + \
                                    ' --w 400 --h 400 --synch'
                    writeModule(xmlFile=xmlFile, name='yarpview', parameters=paramsViewer)
                    
                    fromPort = '/file/' + channelAndDataType + ':o'
                    toPort = '/framer' + channelAndDataType + '/' + channelAndDataType + '/IMUS:i'
                    writeConnection(xmlFile=xmlFile, fromPort=fromPort, toPort=toPort)
                    
                    fromPort = '/framer' + channelAndDataType + '/' + channelAndDataType + '/image:o'
                    toPort = '/viewer/' + channelAndDataType 
                    writeConnection(xmlFile=xmlFile, fromPort=fromPort, toPort=toPort)
                    
                    
        xmlFile.write('</application>\n')

def encodeEvents24Bit(ts, x, y, pol, **kwargs):
    # timestamps(ts) are 32 bit integers counted with an 80 ns clock. 
    # Events are encoded as 32 bits with x,y,channel(ch)(c) and polarity(pol)(p) as shown below
    # 0000 0000 tcrr yyyy yyyy rrxx xxxx xxxp    (r = reserved)
    # t = 0 to indicate events
    # Ignoring channel though
    ts = ts / 0.00000008
    ts = ts.astype(np.uint32) # Timestamp wrapping occurs here
    ts = np.expand_dims(ts, 1)
    x = x.astype(np.int32)
    y = y.astype(np.int32)
    pol = (~pol).astype(np.int32)
    channel = kwargs.get('channel', 0)
    ae = (channel << 22) + (y << 12) + (x << 1) + pol
    ae = np.expand_dims(ae, 1)
    data = np.concatenate([ts, ae], axis=1)
    data = data.flatten().tolist()
    data = list(map(str, data))
    return data

def exportDvs(dataFile, data, bottleNumber, **kwargs):
    # preconvert numpy arrays into bottle-ready data
    print('Converting event arrays to bottle-ready string format ...')
    eventsAsListOfStrings = encodeEvents24Bit(data['ts'], 
                                              data['x'],
                                              data['y'],
                                              data['pol'],
                                              **kwargs)
    print('Breaking into bottles by time')
    # Output dvs data bottle by bottle.
    minTimeStepPerBottle = kwargs.get('minTimeStepPerBottle', 2e-3)
    numEvents = len(data['ts'])
    ptr = 0
    pbar = tqdm(total=numEvents, position=0, leave=True)
    bottleStrs = []
    while ptr < numEvents:
        firstTs = data['ts'][ptr]
        nextPtr = np.searchsorted(data['ts'], firstTs + minTimeStepPerBottle)
        pbar.update(nextPtr-ptr)
        bottleStrs.append(str(bottleNumber) + ' ' + 
                       '{0:.6f}'.format(firstTs) + ' AE (' +
                       ' '.join(eventsAsListOfStrings[ptr*2 : nextPtr*2]) + ')')
        ptr = nextPtr
        bottleNumber += 1 
    dataFile.write('\n'.join(bottleStrs))

def exportFrame(dataFile, data, **kwargs):
    # Here is an example from the python camera - rgb though:
    # 1368 1571751370.660680 [mat] [rgb] (3 230400 8 320 240) {45 43 ... }
    lineStrs = []
    for bottleNum, tsFrameTuple in enumerate(tqdm(list(zip(data['ts'], data['frames'])), position=0, leave=True)):
        ts, frame = tsFrameTuple
        xDim, yDim = frame.shape
        lineStrs.append(str(bottleNum + 1) + ' ' + "{0:.3f}".format(ts) + ' [mat] [mono] (' + 
            ' '.join(['1', str(frame.size), '8', str((frame.shape)[0]), str((frame.shape)[1])]) + ') {' +
            ' '.join(list(map(str, frame.flatten().tolist()))) + '}\n')

        ''' expand mono to rgb
        frameRgb = np.reshape(frame, (-1, 1))
        frameRgb = (frameRgb.astype('float64') * 99 / 255).astype('uint8')  
        frameRgb = np.concatenate((frameRgb, frameRgb, frameRgb), axis=1).flatten()
        frameRgb = np.random.randint(low=0, high=100, size=129600, dtype=np.uint8)
        lineStrs.append(str(bottleNum + 1) + ' ' + "{0:.6f}".format(ts) + ' [mat] [rgb] (' + 
            ' '.join(['3', str(frame.size * 3), '8', str((frame.shape)[0]), str((frame.shape)[1])]) + ') {' +
            ' '.join(list(map(str, frameRgb.tolist()))) + '}\n')
        '''
    dataFile.write(''.join(lineStrs))

def exportPose6q(dataFile, data, **kwargs):
    # Following Prashanth's lead here:  https://github.com/robotology/whole-body-estimators/blob/b55be34ce0abab46a14b7bb25d39d9ede5d05d0e/devices/baseEstimatorV1/include/baseEstimatorV1.h#L421
    pass

def encodeImu(ts, **kwargs):
    # An IMU reading should go in a bottle with 10 consecutive events.
    # Bottles may contain multiple packets like this
    # An event is a 4-byte ts followed by a 4-byte sample
    # timestamps(ts) are 32 bit integers counted with an 80 ns clock. 
    # Events are encoded as 32 bits with value(v), sensor(s), channel(c), and typet) as shown below
    # rrrr rrrr tcrr ssss vvvv vvvv vvvv vvvv    (r = reserved = 0)
    # Ignoring channel though
    ''' Struct of imu sample is (from LSB to MSB):
    int value:16;
    unsigned int sensor:4; - see below ...
    unsigned int _r1:2; - reserved
    unsigned int channel:1; - left vs right
    unsigned int type:1; - should be 1 to indicate sample (cf event)
    unsigned int _r2:8; - reserved
    'Sensor' is defined as: 
    0: Accel -Y
    1: Accel X
    2: Accel Z
    3: Gyro -Y
    4: Gyro X
    5: Gyro Z
    6: Temperature
    7: Mag -Y
    8: Mag X
    9: Mag Z
    
    For the IMU on STEFI, which is this one:
    ICM-20648, gyro full scale is +/-2000 degrees per second,
    accelerometer full scale is +/-2 g.    
    temp - 333.87 - but does it zero at 0K or at room temperature? (+21deg)
    mag - no idea - this model doesn't have an internal mag
    '''
    numSamples = len(ts)
    accConversionFactor = kwargs.get('accConversionFactor', 32768 / 2 / 9.80665)
    angVConversionFactor = kwargs.get('angVConversionFactor', 32768 / 2000 * 180 / np.pi)
    tempConversionFactor = kwargs.get('tempConversionFactor', 333.87)
    tempConversionOffset = kwargs.get('tempConversionOffset', -273.15 - 21)
    magConversionFactor = kwargs.get('magConversionFactor', 1)
    
    acc = kwargs.get('acc')
    angV = kwargs.get('angV')
    temp = kwargs.get('temp')
    mag = kwargs.get('mag')
    
    acc = np.zeros((numSamples, 3), dtype = np.int16) if acc is None else (acc * accConversionFactor).astype(np.int16)
    angV = np.zeros((numSamples, 3), dtype = np.int16) if angV is None else (angV * angVConversionFactor).astype(np.int16)
    temp = np.zeros((numSamples, 1), dtype = np.int16) if temp is None else ((temp + tempConversionOffset) * tempConversionFactor).astype(np.int16)
    mag = np.zeros((numSamples, 3), dtype = np.int16) if mag is None else (mag * magConversionFactor).astype(np.int16)
    
    # switch X and Y; negate Y, to match IMU as mounted on Stefi
    acc = acc[:, [1, 0, 2]]
    angV = angV[:, [1, 0, 2]]
    mag = mag[:, [1, 0, 2]]
    acc[:, 0] = - acc[:, 0]
    angV[:, 0] = - angV[:, 0]
    mag[:, 0] = - mag[:, 0]
    
    ts = ts / 0.00000008
    ts = ts.astype(np.uint32) # Timestamp wrapping occurs here
    ts = np.tile(ts, (1, 10))
    ts = ts.flatten(order='C')
    ts = np.expand_dims(ts, 1)
    values = np.concatenate((acc, angV, temp, mag), axis=1)
    values = values.flatten(order='C')
    sensor = np.tile(np.arange(10, dtype=np.uint32), (numSamples, 1))
    sensor = sensor.flatten(order='C')
    channel = kwargs.get('channel', 0)
    eventType = 1
    ae = (eventType << 23) + (channel << 22) + (sensor << 16) + values 
    ae = np.expand_dims(ae, 1)
    data = np.concatenate([ts, ae], axis=1)
    data = data.flatten().tolist()
    data = list(map(str, data))
    return data
      
def exportImu(dataFile, data, bottleNumber, **kwargs):
    # preconvert numpy arrays into bottle-ready data
    print('Converting event arrays to bottle-ready string format ...')
    eventsAsListOfStrings = encodeImu(ts=data['ts'], 
                                      acc=data['acc'],
                                      angV=data['angV'])
    print('Breaking into bottles by time')
    # Output dvs data bottle by bottle.
    minTimeStepPerBottle = kwargs.get('minTimeStepPerBottle', 2e-3)
    numEvents = len(data['ts'])
    ptr = 0
    pbar = tqdm(total=numEvents, position=0, leave=True)
    bottleStrs = []
    while ptr < numEvents:
        firstTs = data['ts'][ptr]
        nextPtr = np.searchsorted(data['ts'], firstTs + minTimeStepPerBottle)
        pbar.update(nextPtr-ptr)
        # Why 20 in the following? Because there are 10 samples per imu and 2 ints per sample that need to be included. 
        bottleStrs.append(str(bottleNumber) + ' ' + 
                       '{0:.6f}'.format(firstTs) + ' IMUS (' +
                       ' '.join(eventsAsListOfStrings[ptr*20 : nextPtr*20]) + ')')
        ptr = nextPtr
        bottleNumber += 1 
    dataFile.write('\n'.join(bottleStrs))

def encodeSample(data, **kwargs):
    # Samples are encoded like this
    # An event is a 4-byte ts followed by a 4-byte sample
    # timestamps(ts) are 32 bit integers counted with an 80 ns clock. 
    # Events are encoded as 32 bits with value(v), sensor(s), channel(c), and typet) as shown below
    # rrrr rrrr tcrr ssss vvvv vvvv vvvv vvvv    (r = reserved = 0)
    # Ignoring channel though
    
    #Unpack
    ts = data['ts']
    sensor = data['sensor'].astype(np.uint32) # change type to allow shifting up bits
    value = data['value'].astype(np.uint16) # change to unsigned int to avoid wrapping in the sum below
    ts = ts / 0.00000008
    ts = ts.astype(np.uint32) # Timestamp wrapping occurs here
    ts = np.expand_dims(ts, 1)
    channel = kwargs.get('channel', 0)
    eventType = 1
    ae = (eventType << 23) + (channel << 22) + (sensor << 16) + value
    ae = np.expand_dims(ae, 1)
    data = np.concatenate([ts, ae], axis=1)
    data = data.flatten().tolist()
    data = list(map(str, data))
    return data
      

def exportSample(dataFile, data, bottleNumber, **kwargs):
    # preconvert numpy arrays into bottle-ready data
    print('Converting event arrays to bottle-ready string format ...')
    eventsAsListOfStrings = encodeSample(data, **kwargs)
    print('Breaking into bottles by time')
    # Output dvs data bottle by bottle.
    minTimeStepPerBottle = kwargs.get('minTimeStepPerBottle', 2e-3)
    numEvents = len(data['ts'])
    ptr = 0
    pbar = tqdm(total=numEvents, position=0, leave=True)
    bottleStrs = []
    while ptr < numEvents:
        firstTs = data['ts'][ptr]
        nextPtr = np.searchsorted(data['ts'], firstTs + minTimeStepPerBottle)
        pbar.update(nextPtr-ptr)
        # Why 20 in the following? Because there are 10 samples per imu and 2 ints per sample that need to be included. 
        bottleStrs.append(str(bottleNumber) + ' ' + 
                       '{0:.6f}'.format(firstTs) + ' IMUS (' +
                       ' '.join(eventsAsListOfStrings[ptr*2 : nextPtr*2]) + ')')
        ptr = nextPtr
        bottleNumber += 1 
    dataFile.write('\n'.join(bottleStrs))
      
def exportIitYarp(importedDict, **kwargs):
    exportFilePath = kwargs.get('exportFilePath', './')
    if isinstance(importedDict, list):
        for idx, elem in enumerate(importedDict):
            kwargs['exportFilePath'] = exportFilePath + '/' + str(idx)
            exportIitYarp(elem, **kwargs)
        return
    print('exportIitYarp called for data imported from ' + 
          str(importedDict['info']) +
          ' targeting folder ' + exportFilePath)
    try:
        os.makedirs(exportFilePath)
    except FileExistsError: # The folder already exists
        pass
    yarpOutputPort = kwargs.get('yarpOutputPort', '/file')
    channelNames = importedDict['data'].keys()
    dataTypesToExport = kwargs.get('dataTypes')
    for channelName in channelNames:
        if channelName in ['1', 'right']:
            kwargs['channel'] = 1
        else:
            kwargs['channel'] = 0
        dataTypes = importedDict['data'][channelName].keys()
        for dataType in dataTypes:
            if dataTypesToExport is None or dataType in dataTypesToExport: # , 'pose6', 'imu'                
                if dataType not in ['dvs', 'frame', 'imu', 'sample']: 
                    print('unknown datatype')
                    continue # Exclude unknown datatypes
                channelNameAndDataType = channelName + dataType # This is used as a new channel name
                channelPath = os.path.join(exportFilePath, str(channelNameAndDataType))
                try:
                    os.mkdir(channelPath)
                except FileExistsError: # The folder already exists
                    pass
                dirList = os.listdir(channelPath)

                writeMode = kwargs.get('writeMode', 'w')
                if 'data.log' in dirList:
                    print('data already exists in export directory for channel and datatype' + str(channelNameAndDataType))
                    if kwargs.get('protectedWrite', True):
                        print('export not performed')
                        continue
                    else:
                        if writeMode == 'w':
                            print('data is being overwritten!')
                        elif writeMode == 'a':
                            print('data is being appended!')
                            with open(os.path.join(channelPath, 'data.log'), 'r') as f:
                                lines = f.read().splitlines()
                                bottleNumberStart = len(lines)

                # Write the info.log file
                yarpOutputPortForChannel = yarpOutputPort + '/' + channelNameAndDataType + ":o"
                with open(os.path.join(channelPath, 'info.log'), 'w') as infoFile:
                    infoFile.write('Type: Bottle;\n')
                    # TODO: get the real start time from the 'info', if it exists 
                    # - placeholder 0.0 put in the following line 
                    infoFile.write('[0.0] ' + yarpOutputPortForChannel + ' [connected]\n')
                # Write the data.log file
                with open(os.path.join(channelPath, 'data.log'), writeMode) as dataFile:
                    if writeMode == 'a':
                        dataFile.write('\n')
                    data = importedDict['data'][channelName][dataType]
                    if dataType == 'dvs':
                        if writeMode == 'a':
                             exportDvs(dataFile, data, bottleNumberStart, **kwargs)
                        else:
                            exportDvs(dataFile, data, 0, **kwargs)
                    elif dataType == 'frame':
                        # TODO: handle bottle numbering for writeMode = 'a'
                        exportFrame(dataFile, data, **kwargs)
                    elif dataType == 'imu':
                        if writeMode == 'a':
                            exportImu(dataFile, data, bottleNumberStart, **kwargs)
                        else:
                            exportImu(dataFile, data, 0, **kwargs)
                    elif dataType == 'pose6q':
                        # TODO: handle bottle numbering for writeMode = 'a'
                        exportPose6q(dataFile, data, **kwargs)
                    elif dataType == 'sample':
                        if writeMode == 'a':
                            exportSample(dataFile, data, bottleNumberStart, **kwargs)
                        else:
                            exportSample(dataFile, data, 0, **kwargs)
            else:
                print("datatype: ", dataType, " not handled yet")
    if kwargs.get('viewerApp', True):
        exportIitYarpViewer(importedDict, **kwargs)