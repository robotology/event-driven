import numpy as np
import os
import gzip

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


    gt_filename = os.path.join(os.path.dirname(filePathOrName), 'boxes.npy')

    if os.path.exists(gt_filename):
        gt = np.load(gt_filename)
        tsOffset = min(tsOffset, gt[0, 0])
        gt[:, 0] -= tsOffset
        outDict['data']['ch']['dvs']['gt_bb'] = {}
        outDict['data']['ch']['dvs']['gt_bb']['ts'] = gt[:, 0]
        outDict['data']['ch']['dvs']['gt_bb']['minY'] = gt[:, 1]
        outDict['data']['ch']['dvs']['gt_bb']['minX'] = gt[:, 2]
        outDict['data']['ch']['dvs']['gt_bb']['maxY'] = gt[:, 3]
        outDict['data']['ch']['dvs']['gt_bb']['maxX'] = gt[:, 4]
        outDict['data']['ch']['dvs']['gt_bb']['label'] = gt[:, 5]

    events[:, 0] -= tsOffset

    outDict['data']['ch']['dvs']['tsOffset'] = tsOffset
    outDict['data']['ch']['dvs']['ts'] = events[:, 0]
    outDict['data']['ch']['dvs']['x'] = events[:, 1].astype(np.int)
    outDict['data']['ch']['dvs']['y'] = events[:, 2].astype(np.int)
    outDict['data']['ch']['dvs']['pol'] = events[:, 3].astype(np.bool)
    outDict['data']['ch']['dvs']['dimX'] = 304
    outDict['data']['ch']['dvs']['dimY'] = 240
    return outDict