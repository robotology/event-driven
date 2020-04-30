import numpy as np
import os

def importNumpy(filePathOrName, **kwargs):
    outDict = {
        'info': kwargs,
        'data': {}
    }
    outDict['info']['filePathOrName'] = filePathOrName

    events = np.load(filePathOrName)

    outDict['data']['ch'] = {}
    outDict['data']['ch']['dvs'] = {}
    tsOffset = events[0, 0]
    ts_to_sync = [events[:, 0]]

    gt_filename = os.path.join(os.path.dirname(filePathOrName), 'boxes.npy')

    if os.path.exists(gt_filename):
        b_boxes = np.load(gt_filename)
        tsOffset = min(tsOffset, b_boxes[0, 0])
        ts_to_sync.append(b_boxes[:, 0])
        outDict['data']['ch']['dvs']['b_boxes'] = {}
        outDict['data']['ch']['dvs']['b_boxes']['ts'] = b_boxes[:, 0]
        outDict['data']['ch']['dvs']['b_boxes']['minY'] = b_boxes[:, 1]
        outDict['data']['ch']['dvs']['b_boxes']['minX'] = b_boxes[:, 2]
        outDict['data']['ch']['dvs']['b_boxes']['maxY'] = b_boxes[:, 3]
        outDict['data']['ch']['dvs']['b_boxes']['maxX'] = b_boxes[:, 4]
        outDict['data']['ch']['dvs']['b_boxes']['label'] = b_boxes[:, 5]

    framesPath = (os.path.join(os.path.dirname(filePathOrName), '../processed/frames_left'))
    if os.path.exists(framesPath):
        import re
        from matplotlib.pyplot import imread
        pattern = re.compile('\d+ (\d+\.\d+) (.+\.\w+) \[rgb\]')
        with open(os.path.join(framesPath, 'data.log')) as f:
            content = f.read()
            found = np.array(pattern.findall(content))

        frames_ts = found[:, 0].astype(np.float)
        tsOffset = min(tsOffset, frames_ts[0])
        ts_to_sync.append(frames_ts)
        outDict['data']['ch']['frame'] = {}
        outDict['data']['ch']['frame']['ts'] = frames_ts
        outDict['data']['ch']['frame']['frames'] = [imread(os.path.join(framesPath, x)) for x in found[:, 1]]


    framesPath = (os.path.join(os.path.dirname(filePathOrName), '../processed/gt_left'))
    if os.path.exists(framesPath):
        import re
        pattern = re.compile('\d+ (\d+\.\d+) (\d+\.\w+) (.*)')
        pattern2 = re.compile('\( (\d+ \d+ \d+ \d+) \) (\d+) (\d+\.\d+)')
        with open(os.path.join(framesPath, 'data.log')) as f:
            content = f.read()
            found = np.array(pattern.findall(content))

        #boxes_ts = found[:, 0].astype(np.float)

        #outDict['data']['ch']['frame']['bboxes']['frames'] = [imread(os.path.join(framesPath, x)) for x in found[:, 1]]
        outDict['data']['ch']['frame']['b_boxes'] = {}
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
        outDict['data']['ch']['frame']['b_boxes']['ts'] = boxes_ts
        outDict['data']['ch']['frame']['b_boxes']['minY'] = boxes[:, 0]
        outDict['data']['ch']['frame']['b_boxes']['minX'] = boxes[:, 1]
        outDict['data']['ch']['frame']['b_boxes']['maxY'] = boxes[:, 2]
        outDict['data']['ch']['frame']['b_boxes']['maxX'] = boxes[:, 3]
        outDict['data']['ch']['frame']['b_boxes']['label'] = labels

        tsOffset = min(tsOffset, boxes_ts[0])
        ts_to_sync.append(boxes_ts)

    for x in ts_to_sync:
        x -= tsOffset

    outDict['data']['ch']['dvs']['tsOffset'] = tsOffset
    outDict['data']['ch']['dvs']['ts'] = events[:, 0]
    outDict['data']['ch']['dvs']['x'] = events[:, 1].astype(np.int)
    outDict['data']['ch']['dvs']['y'] = events[:, 2].astype(np.int)
    outDict['data']['ch']['dvs']['pol'] = events[:, 3].astype(np.bool)
    outDict['data']['ch']['dvs']['dimX'] = 304
    outDict['data']['ch']['dvs']['dimY'] = 240

    return outDict