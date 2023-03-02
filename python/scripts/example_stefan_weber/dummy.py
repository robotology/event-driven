#!/usr/bin/env python3

# Author: Stefan Weber
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

import yarp
import numpy as np
import vBuffer

yarp.Network.init()

bottleBuffer = vBuffer.VBottleBuffer(5000, "/dummy:i")
with bottleBuffer as buf:
    po = yarp.BufferedPortBottle()
    po.open("/dummy:o")
    i = 0
    ##setup
    yarp_image = yarp.ImageRgb()

    img_array = np.random.uniform(0., 255., (240, 320, 3)).astype(
        np.uint8)  # Create the yarp image and wrap it around the array self.yarp_image = yarp.ImageRgb() self.yarp_image.setExternal(self.img_array, self.img_array.shape[1], self.img_array.shape[0]) # Create the yarp port, connect it to the running instance of yarpview and send the image self.output_port = yarp.Port()
    yarp_image.setExternal(img_array, img_array.shape[1],
                                img_array.shape[0])
    output_port = yarp.Port()
    output_port.open("/obj_det/img:o")
    ##writing loop



    while True:
        i += 1
        if (i%10 == 0):
            continue
        img_array[:] = np.random.uniform(0., 255., (240, 320, 3)).astype(np.uint8)
        output_port.write(yarp_image)
        data = bottleBuffer.timeFrameQueue.get()
        bout = po.prepare()
        bout.clear()
        # import ipdb; ipdb.set_trace()
        val = int(data[-1,1])
        print(val)
        bout.addInt32(val)
        print('writing %s' % val)
        po.write()
    output_port.close()
