# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Sim Bamford
Code contributions from: 
        marco monforte
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importIitYarp opens a data.log file, initially assumed to contain 24-bit encoded data from ATIS.
Returns a list of dicts, where each dict contains
{'info': {},
 'data': {
         channel0: {} (usually the 'left' channel)
         channel1: {} (usually the 'right' channel)
         ...
         }}

Each channel is a dict containing:
    For dvs:
        - "ts": numpy array of float - seconds
        - "x": numpy array of uint16
        - "y": numpy array of uint16
        - "pol": numpy array of uint8 in [0, 1]
    For imu:
        - "ts": numpy array of float - seconds

    Can also handle LAE (labelled event) bottles, IMU

This function combines the import of two distinctly different cases:
    1) pre-split output directly from zynqGrabber
    2) split and processed events from vPreProcess or other downstream modules
The former may combine samples and events in the same bottles and never contains
explicit IMU / LAE / FLOW events. samples such as IMU are contained within AE type bottles.
The latter usually has distinct bottles for each datatype, and channels
have probably, but not definitely, been split into left and right. 
"""
# TODO: get initial ts from yarp info.log 
# TODO: support LAE import
# TODO: support IMU, Skin etc import

#%%

import re
import numpy as np
import os
from tqdm import tqdm

# local imports

from split import splitByLabelled
from timestamps import unwrapTimestamps, zeroTimestampsForAChannel, rezeroTimestampsForImportedDicts

pattern = re.compile('(\d+) (\d+\.\d+) ([A-Z]+) \((.*)\)')


def decodeEvents24Bit(data):
    '''
    timestamps(ts) are 32 bit integers counted with an 80 ns clock - do this conversion later 
    Events are encoded as 32 bits with x,y,channel(ch)(c) and polarity(pol)(p) as shown below
    0000 0000 tcrr yyyy yyyy rrxx xxxx xxxp
    We could pick out the following event types as follows:
    data[:, 1] >>= 1
    aps  = np.uint8(data[:, 1] & 0x01)
    data[:, 1] >>= 1
    skin  = np.uint8(data[:, 1] & 0x01)    
    data[:, 1] >>= 1
    imu  = np.uint8(data[:, 1] & 0x01)    
    data[:, 1] >>= 1
    audio  = np.uint8(data[:, 1] & 0x01)
    '''    
    isNotDvsEvent = np.bool_(data[:, 1] & 0xFF800000)
    if np.any(isNotDvsEvent):
        dataSample = data[isNotDvsEvent, :]
        #ts = np.float64(dataSample[:, 0] & ~(0x1 << 31)) * 0.00000008 # convert ts to seconds
        ts = np.uint32(dataSample[:, 0])
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts    
        ''' For now assuming that sample is IMU - later need to consider other samples: skin, audio etc
        Struct of imu sample is (from MSB left to LSB right):
        rrrr rrrr tcrr ssss vvvv vvvv vvvv vvvv
        r = reserved = 0
        t = type: 1=sample
        c = channel (left vs right)
        s = sensor (uint4)
        v = value of the sample (int16)
        '''
        value = np.int16(dataSample[:, 1] & 0xFFFF)    
        dataSample[:, 1] >>= 16
        sensor = np.uint8(dataSample[:, 1] & 0x0F)    
        dataSample[:, 1] >>= 6
        ch  = np.uint8(dataSample[:, 1] & 0x01)
        samplesOut = {
                'ts': ts, 
                'ch': ch, 
                'sensor': sensor, 
                'value': value
            }
    else:
        samplesOut = None
    if np.any(~isNotDvsEvent):
        dataDvs = data[~isNotDvsEvent, :]
        # convert ts to seconds
        #ts = np.float64(dataDvs[:, 0] & ~(0x1 << 31)) * 0.00000008 # convert ts to seconds
        ts = np.uint32(dataDvs[:, 0])
        if np.isscalar(ts):
            ts = np.ones((1), dtype=np.uint32) * ts    
        pol  = np.bool_(dataDvs[:, 1] & 0x01)    
        dataDvs[:, 1] >>= 1
        x = np.uint16(dataDvs[:, 1] & 0x1FF)    
        dataDvs[:, 1] >>= 11
        y = np.uint16(dataDvs[:, 1] & 0xFF)    
        dataDvs[:, 1] >>= 10
        ch  = np.uint8(dataDvs[:, 1] & 0x01)
        dvsOut = {
                'ts': ts, 
                'ch': ch, 
                'x': x, 
                'y': y,
                'pol': pol
            }
    else:
        dvsOut = None
    return dvsOut, samplesOut
    
def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value

def samplesToImu(inDict, **kwargs):
    '''
    Input is a dict containing ts, s, and v.
    Assume these are IMU samples, and compile them (up to) 10 at a time.
    Output is ts, acc, angV, temp and mag
    'Sensor' is defined as:
    0: Accel X
    1: Accel Y
    2: Accel Z
    3: Gyro X
    4: Gyro Y
    5: Gyro Z
    6: Temperature
    7: Mag X
    8: Mag Y
    9: Mag Z       
    For the IMU on STEFI, which is this one:
    ICM-20648, gyro full scale is +/-2000 degrees per second,
    accelerometer full scale is +/-2 g.   
    temp - 333.87 - but does it zero at 0K or at room temperature? (+21deg)
    mag - no idea - this model doesn't have an internal mag        
    '''
    accConversionFactor = kwargs.get('accConversionFactor', 32768 / 2 / 9.80665)
    angVConversionFactor = kwargs.get('angVConversionFactor', 32768 / 2000 * 180 / np.pi)
    tempConversionFactor = kwargs.get('tempConversionFactor', 333.87)
    tempConversionOffset = kwargs.get('tempConversionOffset', -273.15 - 21)
    magConversionFactor = kwargs.get('magConversionFactor', 1)
    # Assume that sensor always ramps up, and split when it wraps around
    # Otherwise, iterate through this sample by sample - slowest and most robust method
    # Possible to do this much faster, but only my assuming no gaps in data
    tsAll = inDict['ts']
    sensorAll = inDict['sensor'].astype(np.int16)
    valueAll = inDict['value']
    wrapIds = np.where((sensorAll[1:]-sensorAll[:-1])<1)[0]
    numImu = len(wrapIds) + 1
    tsOut = np.zeros((numImu, 1), dtype=np.uint32)
    acc = np.zeros((numImu, 3), dtype=np.int16)
    angV = np.zeros((numImu, 3), dtype=np.int16)
    temp = np.zeros((numImu, 1), dtype=np.int16)
    mag = np.zeros((numImu, 3), dtype=np.int16)
    imuPtr = -1
    sPrev = 100
    for ts, s, v in zip(tsAll, sensorAll, valueAll):
        if s <= sPrev:
            imuPtr += 1
            tsOut[imuPtr] = ts # Just take the first ts for a group of samples
        sPrev = s
        if s == 0:
            acc[imuPtr, 0] = v
        elif s== 1:
            acc[imuPtr, 1] = v
        elif s== 2:
            acc[imuPtr, 2] = v
        elif s== 3:
            angV[imuPtr, 0] = v
        elif s== 4:
            angV[imuPtr, 1] = v
        elif s== 5:
            angV[imuPtr, 2] = v
        elif s== 6:
            temp[imuPtr, 0] = v
        elif s== 7:
            mag[imuPtr, 0] = v
        elif s== 8:
            mag[imuPtr, 1] = v
        elif s== 9:
            mag[imuPtr, 2] = v
    acc = acc.astype(np.float64) / accConversionFactor
    angV = angV.astype(np.float64) / angVConversionFactor
    temp = temp.astype(np.float64) / tempConversionFactor - tempConversionOffset
    mag = mag.astype(np.float64) / magConversionFactor
    outDict = {
            'ts': tsOut,
            'acc': acc,
            'angV': angV,
            'temp': temp,
            'mag': mag,
            }
    return outDict

def importIitYarpBinHavingFoundFile(**kwargs):
    with open(kwargs['filePathAndName'], 'rb') as inFile:
        events = inFile.read()
    events = np.frombuffer(events, np.uint32)
    events = events.reshape((-1, 2))
    dvs, samples = decodeEvents24Bit(events)
    return importPostProcessing(dvs, samples, dvslbl=None, flow=None, **kwargs)

# Sample is an intermediate data type - later it gets converted to IMU etc
def createDataTypeSample():
    sizeOfArray = 1024
    return {
        'ts': np.zeros((sizeOfArray), dtype=np.uint32),
        'ch': np.zeros((sizeOfArray), dtype=np.uint8),
        'sensor': np.zeros((sizeOfArray), dtype=np.uint8),
        'value': np.zeros((sizeOfArray), dtype=np.int16),
        'numEvents': 0
    }   
       
def createDataTypeDvs():
    sizeOfArray = 1024
    return {
        'ts': np.zeros((sizeOfArray), dtype=np.uint32),
        'ch': np.zeros((sizeOfArray), dtype=np.uint8),
        'x': np.zeros((sizeOfArray), dtype=np.uint16),
        'y': np.zeros((sizeOfArray), dtype=np.uint16),
        'pol': np.zeros((sizeOfArray), dtype=np.bool),
        'numEvents': 0 
        }
                    
def createDataTypeDvslbl():
    dvs = createDataTypeDvs()
    sizeOfArray = len(dvs['ts'])
    dvs['lbl'] = np.zeros((sizeOfArray), dtype=np.int64)
    return dvs
                    
def createDataTypeDvsFlow():
    dvs = createDataTypeDvs()
    sizeOfArray = len(dvs['ts'])
    dvs['vx'] = np.zeros((sizeOfArray), dtype=np.float64)
    dvs['vy'] = np.zeros((sizeOfArray), dtype=np.float64)
    return dvs
                
def appendBatch(mainDict, batch):
    if batch is None: return mainDict
    # Check if the main array has enough free space for this batch
    sizeOfMain = len(mainDict['ts'])
    numEventsInMain = mainDict['numEvents']
    numEventsInBatch = len(batch['ts'])
    while numEventsInBatch + numEventsInMain > sizeOfMain:
        for fieldName in mainDict.keys():
            if fieldName != 'numEvents':
                # Double the size of the array
                mainDict[fieldName] = np.append(mainDict[fieldName], np.empty_like(mainDict[fieldName]))
        sizeOfMain *= 2
    # Add the batch into the main array
    for fieldName in batch.keys():
        mainDict[fieldName][numEventsInMain:numEventsInMain + numEventsInBatch] = \
            batch[fieldName]
    mainDict['numEvents'] = numEventsInMain + numEventsInBatch
    return mainDict
 
def cropArraysToNumEvents(inDict):
    numEvents = inDict.pop('numEvents')
    for fieldName in inDict.keys():
        inDict[fieldName] = inDict[fieldName][:numEvents]

def selectByChannel(inDict, channelLabel):
    selectedEvents = inDict['ch'] == channelLabel
    if not np.any(selectedEvents):
        return None    
    outDict = {}
    for fieldName in inDict:
        if fieldName != 'ch':
            outDict[fieldName] = inDict[fieldName][selectedEvents]
    return outDict
            
'''
Having imported data from the either a datadumper log or a bin file, 
carry out common post-processing steps: 
    convert samples to IMU (etc)
    split by channel
    unwrap timestamps
'''
def importPostProcessing(dvs, samples, dvslbl=None, flow=None, **kwargs):
    '''
    Split by channel: The iit yarp format assumes that the channel bit 
    corresponds to 'left' and 'right' sensors, so it's handled explicitly here
    '''
    channels = {}
    for ch in [0, 1]:
        chDict = {}
        dvsCh = selectByChannel(dvs, ch)
        if dvsCh:
            chDict['dvs'] = dvsCh
        if dvslbl:
            dvslblCh = selectByChannel(dvslbl, ch)
            if dvslblCh:
                chDict['dvslbl'] = dvslblCh
        samplesCh = selectByChannel(samples, ch)
        if samplesCh:
            chDict['imu'] = samplesToImu(samplesCh, **kwargs)
        if any(chDict):
            for dataType in chDict:
                chDict[dataType]['ts'] = unwrapTimestamps(chDict[dataType]['ts'], **kwargs) * 0.00000008 # Convert to seconds
            if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
                zeroTimestampsForAChannel(chDict)
            if ch == 0:
                channels['left'] = chDict
            else:
                channels['right'] = chDict
    # Construct the output dict
    outDict = {
        'info': kwargs,
        'data': channels
        }
    outDict['info']['fileFormat'] = 'iityarp'    
    return outDict

def importIitYarpHavingFoundFile(**kwargs):
    # Create dicts for each possible datatype
    dvs = createDataTypeDvs()
    dvslbl = createDataTypeDvslbl()
    flow = createDataTypeDvsFlow()
    samples = createDataTypeSample() # Sample is an intermediate datatype - later it gets converted to IMU etc
    with open(kwargs['filePathAndName'], 'r') as inFile:
        content = inFile.read()
    found = pattern.findall(content)
    for elem in tqdm(found):
        # The following values would be useful for indexing the input file:
        #bottlenumber = np.uint32(elem[0])
        #timestamp = np.float64(elem[1])
        bottleType = elem[2]
        if bottleType in ['AE', 'LAE', 'IMUS']:
            try:
                events = np.array(elem[3].split(' '), dtype=np.uint32)
                if bottleType == 'LAE':
                    numEventsInBatch = int(len(events) / 3)
                    events = events.reshape(numEventsInBatch, 3)
                    # TODO: finish handling this case
                    dvsBatch, samplesBatch = decodeEvents24Bit(events[:, :2])
                    dvsBatch['lbl'] = events[:, 2]          
                    appendBatch(dvslbl, dvsBatch)
                else:
                    numEventsInBatch = int(len(events) / 2)
                    events = events.reshape(numEventsInBatch, 2)
                    dvsBatch, samplesBatch = decodeEvents24Bit(events[:, :2])
                    appendBatch(dvs, dvsBatch)
                appendBatch(samples, samplesBatch)
            except ValueError: # sometimes finding malformed packets at the end of files - ignoring
                continue
    # Crop arrays to number of events
    cropArraysToNumEvents(dvs)
    cropArraysToNumEvents(dvslbl)
    cropArraysToNumEvents(samples)
    return importPostProcessing(dvs, samples, dvslbl=dvslbl, flow=flow, **kwargs)

def importIitYarpRecursive(**kwargs):
    '''
    This function works in the following way for efficiency when importing 
    very large files:
    - Import events bottle by bottle
    - Create arrays to hold the first 1024 values then extend the array size 
    exponentially while importing, finally cropping these arrays. 
    kwargs is augmented where necessary and becomes the "info" dict of the output.
    '''
    path = getOrInsertDefault(kwargs, 'filePathOrName', '.')
    if not os.path.exists(path):
        raise FileNotFoundError("path not found.")
    if not os.path.isdir(path):
        raise FileNotFoundError("path is not a directory.")
    files = sorted(os.listdir(path))
    importedDicts = []
    for file in files:
        filePathAndName = os.path.join(path, file) 
        if os.path.isdir(filePathAndName):
            kwargs['filePathOrName'] = filePathAndName
            importedDicts = importedDicts + importIitYarpRecursive(**kwargs)
        if file == 'binaryevents.log': # TODO: Placeholder - how shall we denote that this is a binary file?
            kwargs['filePathOrName'] = filePathAndName
            importedDicts.append(importIitYarpBinHavingFoundFile(filePathAndName = filePathAndName, **kwargs))                      
        if file == 'data.log':
            kwargs['filePathOrName'] = filePathAndName
            importedDicts.append(importIitYarpHavingFoundFile(filePathAndName = filePathAndName, **kwargs))          
    if len(importedDicts) == 0:
        raise FileNotFoundError('"data.log" file not found')
    return importedDicts
        
def importIitYarp(**kwargs):
    importedDicts = importIitYarpRecursive(**kwargs)
    if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
        # Optional: start the timestamps at zero for the first event
        # This is done collectively for all the concurrent imports
        rezeroTimestampsForImportedDicts(importedDicts)
    if len(importedDicts) == 1:
        importedDicts = importedDicts[0]
    return importedDicts
    
#%% LEGACY CODE
# Import functions from marco monforte, fro reference
'''
   def decode_skinEvents(self, data):
        # Events are encoded as 32 bits with polarity(p), taxel(t), cross_base(c),
        # body_part(b), side(s), type(T) and skin(S) as shown below. Reserved bits are represented with (R)

        # 0000 0000 STsR RRbb bRRc RRttt tttt tttp

        if data[-1,0] == 0:
            data = np.delete(data, -1)

        timestamps = data[:, 0] & ~(0x1 << 31)

        polarity = data[:, 1] & 0x01
        data[:, 1] >>= 1
        taxel = data[:, 1] & 0x3FF
        data[:, 1] >>= 12
        cross_base = data[:, 1] & 0x01
        data[:, 1] >>= 3
        body_part = data[:, 1] & 0x07
        data[:, 1] >>= 6
        side = data[:, 1] & 0x01
        data[:, 1] >>= 1
        type = data[:, 1] & 0x01
        data[:, 1] >>= 1
        skin = data[:, 1] & 0x01

        return np.vstack([timestamps, polarity, taxel, cross_base, body_part, side, type, skin]).T.astype(np.float)

    def decode_events_bit(self, str_rep):
        # Events are encoded as 32 bits with x,y,channel(c) and polarity(p) as shown below
        # 0000 0000 tcrr yyyy yyyy rrxx xxxx xxxp

        k = list(map(int, str_rep.split()))
        data = np.array(k, dtype=np.uint32)
        # if data[-1] == 0 or len(data) % 2 != 0:
        #     data = np.delete(data, -1)
        data = data.reshape((-1, len(data)))

        timestamps = data[:, 0] & ~(0x1 << 31)

        polarity = data[:, 1] & 0x01

        data[:, 1] >>= 1
        x = data[:, 1] & 0x1FF

        data[:, 1] >>= 11
        y = data[:, 1] & 0xFF

        data[:, 1] >>= 10
        channel = data[:, 1] & 0x01
        return np.vstack([timestamps, channel, x, y, polarity]).T.astype(np.float)


    def load_AE_from_yarp(self, AE_file_path):
        pattern = re.compile('(\d*) (\d*.\d*) AE \((.*)\)')
        AE_to_save = []
        with open(os.path.join(AE_file_path, 'data.log')) as boxFile:
            content = boxFile.read()
            found = pattern.findall(content)
            for b in tqdm(found):
                b_num = b[0]
                b_ts = b[1]
                ev = np.array(b[2].split(' '), dtype=np.uint32)
                ev = ev.reshape(int(len(ev)/2), 2)
                timestamps, channel, x, y, polarity = self.decode_events_24bit(ev)[0]
                AE_to_save.append(np.array([timestamps, channel, x, y, polarity]))       # SELECT WHAT TO SAVE
        AE_to_save = np.array(AE_to_save)
        AE_to_save[:, 0] = (AE_to_save[:, 0] - AE_to_save[0, 0]) * 80e-9  # 80ns to normalize w.r.t. the clock
        np.savetxt(os.path.join(AE_file_path, 'decoded_events.txt'), AE_to_save, delimiter=',', fmt=['%f', '%d', '%d', '%d', '%d'])     # SPECIFY THE FORMAT OF THE DATA


    def load_GAE_from_yarp(self, GAE_file_path):
        pattern = re.compile('(\d*) (\d*.\d*) GAE \((\d*) (\d*) \d{1} (\d*) (\d*) (\d*)\)')
        GAE_to_save = []
        with open(os.path.join(GAE_file_path, 'data.log')) as boxFile:
            content = boxFile.read()
            found = pattern.findall(content)
            for b in found:
                b_num = b[0]
                b_ts = b[1]
                v_ts = b[2]
                v = b[3]
                radius = np.float32(struct.unpack("<f", struct.pack("<I", np.int(
                    b[4]))))  # converts the integer to binary and then to float32
                # tw = struct.unpack("<f", struct.pack("<i", np.int(b[5])))
                # circle = struct.unpack("<f", struct.pack("<i", np.int(b[6])))
                ts, ch, x, y, p = self.decode_events(' '.join([v_ts, v]))[0]  # TODO ONLY WORKS FOR ONE EVENT PER BOTTLE
                GAE_to_save.append(np.array([ts, x, y, radius, p]))
        GAE_to_save = np.array(GAE_to_save)
        GAE_to_save[:, 0] = (GAE_to_save[:, 0] - GAE_to_save[0, 0]) * 80e-9  # 80ns to normalize w.r.t. the clock
        np.savetxt(os.path.join(GAE_file_path, "decoded_events.txt"), GAE_to_save, delimiter=',', fmt=['%f', '%d', '%d', '%d', '%d'])


    def load_SkinEvents_from_yarp(self, AE_file_path):
        pattern = re.compile('(\d*) (\d*.\d*) AE \((.*)\)')
        AE_to_save = []
        with open(os.path.join(AE_file_path, 'data.log')) as boxFile:
            content = boxFile.read()
            found = pattern.findall(content)
            for b in tqdm(found):
                b_num = b[0]
                b_ts = b[1]
                ev = np.array(b[2].split(' '), dtype=np.uint32)
                ev = ev.reshape(int(len(ev)/2), 2)
                ts, pol, tax, cross_b, body_part, side, type, skin = self.decode_skinEvents(ev)[0]
                AE_to_save.append(np.array([ts, pol, tax, cross_b, body_part, side, type, skin]))       # SELECT WHAT TO SAVE
        AE_to_save = np.array(AE_to_save)
        AE_to_save[:, 0] = (AE_to_save[:, 0] - AE_to_save[0, 0]) * 80e-9  # 80ns to normalize w.r.t. the clock
        np.savetxt(os.path.join(AE_file_path, 'decoded_events.txt'), AE_to_save, delimiter=',', fmt=['%f', '%d', '%d', '%d', '%d', '%d', '%d', '%d'])     # SPECIFY THE FORMAT OF THE DATA
'''

    
