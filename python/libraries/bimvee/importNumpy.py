# -*- coding: utf-8 -*-
"""
Copyright (C) 2019 Event-driven Perception for Robotics
Authors: Massimiliano Iacono
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of bimvee (Batch Import, Manipulation, Visualisation and Export of Events etc)
importNumpy is a function for importing timestamped address-event data, given a path to a npy file.
If available at the predetermined locations, relative to the npy file additional data may also be imported, such as
frames and bounding boxes used to label the data. At the moment frames and boxes are stored in a data.log file
using the YARP format (see importIitYarp as an example). TODO generalize data structure

The output is a dictionary containing:
    - info
    - data
The exact contents varies according to the file type import but in general:
    info: this is a dict which starts life as the kwargs passed in, and is
    augmented and modified by the actual contents of the file. It may include
    any informational fields in file headers. Minimally, it will contain:
        - filePathAndName
        - fileFormat
    data: this is a list of dicts, one for each sensor or "channel" which has
    been imported. Bear in mind that sub-functions may optionally split or join
    channels. Within each dict, there is a field for each type of data contained.
    A file for example may contain data from a several sensors, but a single sensor
    may produce polarity events ("pol"), aps samples ("aps"), imu samples etc.
    Within each of these fields, there is a dict, generally containing fields for
    each data column, so in the case of pol events, there are 4-5 fields:
        - ts
        - x
        - y
        - pol
        - optionally ch (channel)
        each containing a numpy array of the appropriate type for the data
        contained, where all these arrays will be of the same length.
    Similarly bounding boxes are saved in the channel in a separate dictionary containing the following fields:
        - ts
        - minY
        - minX
        - maxY
        - maxX
"""

import numpy as np
import os

def importNumpy(filePathOrName, **kwargs):
    outDict = {
        'info': kwargs,
        'data': {}
    }
    outDict['info']['filePathOrName'] = filePathOrName

    # Importing events
    events = np.load(filePathOrName)

    outDict['data']['labelledEvents'] = {}
    outDict['data']['labelledEvents']['dvs'] = {}

    tsOffset = events[0, 0]
    ts_to_sync = [events[:, 0]]

    outDict['data']['labelledEvents']['dvs']['ts'] = events[:, 0]
    outDict['data']['labelledEvents']['dvs']['x'] = events[:, 1].astype(np.int)
    outDict['data']['labelledEvents']['dvs']['y'] = events[:, 2].astype(np.int)
    outDict['data']['labelledEvents']['dvs']['pol'] = events[:, 3].astype(np.bool)
    outDict['data']['labelledEvents']['dvs']['dimX'] = 304
    outDict['data']['labelledEvents']['dvs']['dimY'] = 240


    # Importing bounding boxes for events
    gt_filename = os.path.join(os.path.dirname(filePathOrName), 'boxes.npy')

    if os.path.exists(gt_filename):
        b_boxes = np.load(gt_filename)
        outDict['data']['labelledEvents']['boundingBoxes'] = {}

        tsOffset = min(tsOffset, b_boxes[0, 0])
        ts_to_sync.append(b_boxes[:, 0])
        outDict['data']['labelledEvents']['boundingBoxes']['ts'] = b_boxes[:, 0]
        outDict['data']['labelledEvents']['boundingBoxes']['minY'] = b_boxes[:, 1]
        outDict['data']['labelledEvents']['boundingBoxes']['minX'] = b_boxes[:, 2]
        outDict['data']['labelledEvents']['boundingBoxes']['maxY'] = b_boxes[:, 3]
        outDict['data']['labelledEvents']['boundingBoxes']['maxX'] = b_boxes[:, 4]
        outDict['data']['labelledEvents']['boundingBoxes']['label'] = b_boxes[:, 5]

    # Importing frames
    framesPath = (os.path.join(os.path.dirname(filePathOrName), '../processed/frames_left')) # TODO make path argument
    if os.path.exists(framesPath):
        import re
        from imageio import imread
        pattern = re.compile('\d+ (\d+\.\d+) (.+\.\w+) \[rgb\]')
        with open(os.path.join(framesPath, 'data.log')) as f:
            content = f.read()
            found = np.array(pattern.findall(content))

        outDict['data']['labelledFrames'] = {}
        outDict['data']['labelledFrames']['frame'] = {}

        frames_ts = found[:, 0].astype(np.float)
        tsOffset = min(tsOffset, frames_ts[0])
        ts_to_sync.append(frames_ts)

        outDict['data']['labelledFrames']['frame']['ts'] = frames_ts
        outDict['data']['labelledFrames']['frame']['frames'] = [imread(os.path.join(framesPath, x)) for x in found[:, 1]]


    # Importing Bounding Boxes for frames
    framesPath = (os.path.join(os.path.dirname(filePathOrName), '../processed/gt_left')) # TODO make path argument
    if os.path.exists(framesPath):
        import re
        pattern = re.compile('\d+ (\d+\.\d+) (\d+\.\w+) (.*)') # TODO specific to yarp data format
        pattern2 = re.compile('\( (\d+ \d+ \d+ \d+) \) (\d+) (\d+\.\d+)')
        with open(os.path.join(framesPath, 'data.log')) as f:
            content = f.read()
            found = np.array(pattern.findall(content))

        boxes = []
        boxes_ts = []
        labels = []

        for ts, frame_name, bbs in found:
            for box, label, _ in pattern2.findall(bbs): # TODO third element is confidence score. not used at the moment
                boxes.append(box.split(' '))
                labels.append(label)
                boxes_ts.append(ts)

        boxes_ts = np.array(boxes_ts).astype(np.float)
        labels = np.array(labels).astype(np.int)
        boxes = np.array(boxes).astype(np.int)

        tsOffset = min(tsOffset, boxes_ts[0])
        ts_to_sync.append(boxes_ts)
        outDict['data']['labelledFrames']['boundingBoxes'] = {}
        outDict['data']['labelledFrames']['boundingBoxes']['ts'] = boxes_ts
        outDict['data']['labelledFrames']['boundingBoxes']['minY'] = boxes[:, 0]
        outDict['data']['labelledFrames']['boundingBoxes']['minX'] = boxes[:, 1]
        outDict['data']['labelledFrames']['boundingBoxes']['maxY'] = boxes[:, 2]
        outDict['data']['labelledFrames']['boundingBoxes']['maxX'] = boxes[:, 3]
        outDict['data']['labelledFrames']['boundingBoxes']['label'] = labels

    for x in ts_to_sync:
        x -= tsOffset

    return outDict