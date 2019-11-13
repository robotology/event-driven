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
         channel0: {}
         channel1: {}
         ...
         }}

each channel is a dict containing:
    For dvs:
        - "pol": numpy array of uint8 in [0, 1]
        - "x": numpy array of uint16
        - "y": numpy array of uint16
        - "ts": numpy array of float - seconds
    For imu ...
    Can also handle LAE (labelled event) bottles ...

Under the hood, the algorithm always creates the array form, and then may optionally convert it. 

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
        samplesOut = (ts, ch, sensor, value)
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
        dvsOut = (ts, ch, x, y, pol)
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
    sAll = inDict['s'].astype(np.int16)
    vAll = inDict['v']
    wrapIds = np.where((sAll[1:]-sAll[:-1])<1)[0]
    numImu = len(wrapIds) + 1
    tsOut = np.zeros((numImu, 1), dtype=np.uint32)
    acc = np.zeros((numImu, 3), dtype=np.int16)
    angV = np.zeros((numImu, 3), dtype=np.int16)
    temp = np.zeros((numImu, 1), dtype=np.int16)
    mag = np.zeros((numImu, 3), dtype=np.int16)
    imuPtr = -1
    sPrev = 100
    for ts, s, v in zip(tsAll, sAll, vAll):
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

def importIitYarpHavingFoundFile(**kwargs):
    numEvents = 0
    numSamples = 0
    # dvs data
    sizeOfArrayDvs = 1024
    tsDvs = np.zeros((sizeOfArrayDvs), dtype=np.uint32)
    chDvs = np.zeros((sizeOfArrayDvs), dtype=np.uint8)
    xDvs = np.zeros((sizeOfArrayDvs), dtype=np.uint16)
    yDvs = np.zeros((sizeOfArrayDvs), dtype=np.uint16)
    polDvs = np.zeros((sizeOfArrayDvs), dtype=np.bool)
    lblDvs = np.zeros((sizeOfArrayDvs), dtype=np.int64) - 1 #Tag for not labelled
    # sample data
    sizeOfArraySample = 1024
    tsSample = np.zeros((sizeOfArraySample), dtype=np.uint32)
    chSample = np.zeros((sizeOfArraySample), dtype=np.uint8)
    vSample = np.zeros((sizeOfArraySample), dtype=np.uint16)
    sSample = np.zeros((sizeOfArraySample), dtype=np.uint8)
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
                        lblBatch = events[:, 2]
                    else:
                        numEventsInBatch = int(len(events) / 2)
                        events = events.reshape(numEventsInBatch, 2)
                    dvs, samples = decodeEvents24Bit(events[:, :2])
                    if dvs:
                        tsBatch, chBatch, xBatch, yBatch, polBatch = dvs
                        numEventsInBatch = len(tsBatch)
                        while sizeOfArrayDvs < numEvents + numEventsInBatch:
                            tsDvs = np.append(tsDvs, np.zeros((sizeOfArrayDvs), dtype=np.uint32))
                            chDvs = np.append(chDvs, np.zeros((sizeOfArrayDvs), dtype=np.uint8))
                            xDvs = np.append(xDvs, np.zeros((sizeOfArrayDvs), dtype=np.uint16))
                            yDvs = np.append(yDvs, np.zeros((sizeOfArrayDvs), dtype=np.uint16))
                            polDvs = np.append(polDvs, np.zeros((sizeOfArrayDvs), dtype=np.bool))
                            lblDvs = np.append(lblDvs, np.zeros((sizeOfArrayDvs), dtype=np.int64) - 1)
                            sizeOfArrayDvs *= 2
                        tsDvs[numEvents:numEvents + numEventsInBatch] = tsBatch
                        chDvs[numEvents:numEvents + numEventsInBatch] = chBatch
                        xDvs[numEvents:numEvents + numEventsInBatch] = xBatch
                        yDvs[numEvents:numEvents + numEventsInBatch] = yBatch
                        polDvs[numEvents:numEvents + numEventsInBatch] = polBatch
                        if bottleType == 'LAE':
                            lblDvs[numEvents:numEvents + numEventsInBatch] = lblBatch
                        numEvents += numEventsInBatch        
                    if samples:
                        tsBatch, chBatch, sBatch, vBatch = samples
                        numSamplesInBatch = len(tsBatch)
                        while sizeOfArraySample < numSamples + numSamplesInBatch:
                            tsSample = np.append(tsSample, np.zeros((sizeOfArraySample), dtype=np.uint32))
                            chSample = np.append(chSample, np.zeros((sizeOfArraySample), dtype=np.uint8))
                            sSample = np.append(sSample, np.zeros((sizeOfArraySample), dtype=np.uint16))
                            vSample = np.append(vSample, np.zeros((sizeOfArraySample), dtype=np.uint16))
                            sizeOfArraySample *= 2
                        tsSample[numSamples:numSamples + numSamplesInBatch] = tsBatch
                        chSample[numSamples:numSamples + numSamplesInBatch] = chBatch
                        sSample[numSamples:numSamples + numSamplesInBatch] = sBatch
                        vSample[numSamples:numSamples + numSamplesInBatch] = vBatch
                        if bottleType == 'LAE':
                            raise Exception('Samples in labelled event bottles ...')
                        numSamples += numSamplesInBatch
                except ValueError: # sometimes finding malformed packets at the end of files - ignoring
                    continue
    # Crop arrays to number of events
    tsDvs = tsDvs[:numEvents]
    chDvs = chDvs[:numEvents]
    xDvs = xDvs[:numEvents]
    yDvs = yDvs[:numEvents]
    polDvs = polDvs[:numEvents]
    lblDvs = lblDvs[:numEvents]
    # Crop arrays to number of samples
    tsSample = tsSample[:numSamples]
    chSample = chSample[:numSamples]
    sSample = sSample[:numSamples]
    vSample = vSample[:numSamples]

    '''
    The iit yarp format assumes that the channel bit corresponds to 
    'left' and 'right' sensors, so it's handled explicitly here
    '''
    channels = {}
    for ch in [0, 1]:
        chDict = {}
        selectedEvents = chDvs == ch
        if np.any(selectedEvents):
            chDictDvs = {
                    'lbl': lblDvs[selectedEvents],
                    'ts': tsDvs[selectedEvents],
                    'x': xDvs[selectedEvents],
                    'y': yDvs[selectedEvents],
                    'pol': polDvs[selectedEvents] }
            # If there are labelled data, it will be split out to a separate datatype
            chDict.update(splitByLabelled(chDictDvs))
        selectedSamples = chSample == ch
        if np.any(selectedSamples):
            chDictSamples = {
                    'ts': tsSample[selectedSamples],
                    's': sSample[selectedSamples],
                    'v': vSample[selectedSamples], }
            # If there are labelled data, it will be split out to a separate datatype
            chDict['imu'] = samplesToImu(chDictSamples, **kwargs)
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
    # Split by channel
    # First check if there is any splitting that needs to be done
    return outDict

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

    