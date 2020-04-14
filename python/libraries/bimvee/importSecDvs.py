# -*- coding: utf-8 -*-

"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Sim Bamford
         Ander Arriandiaga Laresgoiti

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importSecDvs opens a .bin file, assumed to contain encoded data from SecDvs gen3.
Returns a dict in this format:
{'info': {},
 'data': {
         ch0: {
               dvs: {
                     'ts': np.array of np.float64 in seconds 
                     'x': np.array of np.uint16 in pixels
                     'y': np.array of np.uint16 in pixels
                     'pol': np.array of np.bool -- 1 = ON event 
                    }}}}
"""

import numpy as np
from tqdm import tqdm 

# Local imports
if __package__ is None or __package__ == '':
    from timestamps import unwrapTimestamps, zeroTimestampsForADataType
else:
    from .timestamps import unwrapTimestamps, zeroTimestampsForADataType


def importSecDvs(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    print('Attempting to import ' + filePathOrName + ' as secdvs')
    with open(filePathOrName, 'rb') as file:
        data = np.fromfile(file, dtype='>u4')

    print('Clipping any rows without column info...')
    # Hunt for the first col iteratively and cut data before this
    for idx, word in enumerate(data):
        if word & 0x4000000:
            break
    data = data[idx:]

    print('Building indices for word types  ...')
    isTs = (data & 0x08000000).astype(np.bool) # Reference timestamp, i.e. tsMsb
    isCol = (data & 0x4000000).astype(np.bool) # X and tsLsb
    isRow = (data & 0x80000000).astype(np.bool) # y and pol

    print('Handling col data ...')
    # create an index which points data back to col
    numCol = np.count_nonzero(isCol)
    colIdsRange = np.arange(numCol)
    colIdsSparse = np.where(isCol)[0]
    colIdsNext = np.append(colIdsSparse[1:], len(data))
    colIdx = np.zeros_like(data)
    for rangeIdx, firstIdx, nextIdx in zip(colIdsRange, colIdsSparse, colIdsNext):
        colIdx[firstIdx:nextIdx] = rangeIdx
    # now we can isolate col
    col = data[isCol]
    # convert col data
    xByCol = (col & 0x000003FF).astype(np.uint16) # column Address
    tsSmallByCol = (col & 0x1FF800) >> 11
	# create col data vectors which match data
    x = xByCol[colIdx]
    tsSmall = tsSmallByCol[colIdx]
    
    print('Handling timestamp data ...')

    # from data extract the "start col" flag
    isStartCol = (data & 0x200000).astype(np.bool) & isCol
    # now, for each startCol, we want to search backwards through data for a ts    
    # To do this, we find the "data" idx of start col, we create a set of tsIds
    # and then we search tsIds for each dataIdx    
    tsIdsSparse = np.where(isTs)[0]
    # Actually find the tsMsb at this point, to avoid more indexing
    tsMsbSparse = (data[isTs] & 0x003FFFFF) << 10
    tsMsbSparse = np.insert(tsMsbSparse, 0, tsMsbSparse[0] - (1 << 10))
    # create an index which points data back to startCols
    startColIdsSparse = np.where(isStartCol)[0]
    # Search tsIds for each dataIdx
    tsForStartCol = np.zeros_like(startColIdsSparse)
    for idx, startColIdx in enumerate(startColIdsSparse):
        tsForStartCol[idx] = tsMsbSparse[np.searchsorted(tsIdsSparse, startColIdx)]

    # Now we have the ts(Large) for each startCol event. Now create a tsLarge
    # array which matches data; do this in just the same way as above for
    # tsSmall given col and colIdx
    # create an index which points data back to col
    numStartCol = np.count_nonzero(isStartCol)
    startColIdsRange = np.arange(numStartCol)
    startColIdsSparse = np.where(isStartCol)[0]
    startColIdsNext = np.append(startColIdsSparse[1:], len(data))
    startColIdx = np.zeros_like(data)
    for rangeIdx, firstIdx, nextIdx in zip(startColIdsRange, startColIdsSparse, startColIdsNext):
        startColIdx[firstIdx:nextIdx] = rangeIdx
    tsLarge = tsForStartCol[startColIdx]

    # Now we have tsLarge and tsSmall aligned by data, 
    # we can sum these to give the full ts
    ts = tsLarge + tsSmall

    print('Handling row data ...')

    # Now we have x and ts for each row in data; 
    # now select these just for group/row data
    x = x[isRow]
    ts = ts[isRow]
    data = data[isRow]

    # A major timewrap is possible, so handle unwrapping before the following 
    # processing, which will mix up the timestamps
    ts = unwrapTimestamps(ts)  / 1000000 # Convert to seconds in the same step
    
    # Break out addr and pol for each of the two groups
    pol1 = ((data & 0x00010000) >> 16).astype(np.bool)
    yLarge1 = ((data & 0x00FC0000) >> 15).astype(np.uint16) # grp1Address
    pol2 = ((data & 0x00020000) >> 17).astype(np.bool)
    grp2Offset = (data & 0x7C000000) >> 23 
    yLarge2 = (grp2Offset + yLarge1).astype(np.uint16)

    # for each of the single bit indices, select events for each of the two groups
    grp1Events = data & 0xFF
    grp2Events = data & 0xFF00
    tsToConcatenate = []
    xToConcatenate = []
    yToConcatenate = []
    polToConcatenate = []
    for idx in tqdm(range(8)):
        #group 1
        grp1Bool = (grp1Events & (2 ** idx)).astype(np.bool)
        tsToConcatenate.append(ts[grp1Bool])
        xToConcatenate.append(x[grp1Bool])
        yToConcatenate.append(yLarge1[grp1Bool] + idx)
        polToConcatenate.append(pol1[grp1Bool])
        # group 2        
        grp2Bool = (grp2Events & (2 ** (idx + 8))).astype(np.bool)
        tsToConcatenate.append(ts[grp2Bool])
        xToConcatenate.append(x[grp2Bool])
        yToConcatenate.append(yLarge2[grp2Bool] + idx)
        polToConcatenate.append(pol2[grp2Bool])

    print('Post-processing steps ...')

    # Concatenate the resulting arrays
    ts = np.concatenate(tsToConcatenate)
    x = np.concatenate(xToConcatenate)
    y = np.concatenate(yToConcatenate)
    pol = np.concatenate(polToConcatenate)
    
    # The above selection strategy mixed up the timestamps, so sort the events by ts
    
    ids = np.argsort(ts)
    
    ts = ts[ids]
    x = x[ids]
    y = y[ids]
    pol = pol[ids]
    
    dvsDict = {'ts': ts,
               'x': x,
               'y': y,
               'pol': pol,
               }

    if kwargs.get('zeroTime', kwargs.get('zeroTimestamps', True)): 
        zeroTimestampsForADataType(dvsDict)
    outDict = {
    'info': {'filePathOrName':filePathOrName,
        'fileFormat': 'secdvs'},
    'data': {
        'ch0': {
            'dvs': dvsDict
            }
        }
    }
    print('Done.')
            
    return outDict
