#!/usr/bin/env python

# --------------------------------------------------------
# yarp RFModule python
# Author: Marco Monforte
# --------------------------------------------------------


import os, re, struct, time, argparse, pickle, collections
import numpy as np

import yarp


# Initialise YARP
yarp.Network.init()



class RFModule(yarp.RFModule):

    def __init__(self, reader_port):

        print('\n\nInitializing predictor...')
        yarp.RFModule.__init__(self)

        # self._pattern = re.compile('(\d*) (\d*.\d*) (\w*) \((.*)\)')

        # Open YARP ports
        print('\nOpening yarp ports...')
        self._reader_port = yarp.BufferedPortBottle()
        self._reader_port.open(reader_port)
        yarp.Network.connect('/zynqGrabber/AE:o', reader_port, 'udp')           # AUTOMATIC CONNECTION - REMOVE IN FUTURE
        print('\nPorts opened successfully!')


    def updateModule(self):
        print('\nModule running happily...')


    def cleanup(self):
        print('\nCleaning up...')
        self._reader_port.close()
        print('\nCleaning completed!')


    def decodeAddressEvents(self, data):
        timestamps = data[:, 0] & ~(0x1 << 31)
        polarity = data[:, 1] & 0x01
        data[:, 1] >>= 1
        x = data[:, 1] & 0x1FF
        data[:, 1] >>= 11
        y = data[:, 1] & 0xFF
        data[:, 1] >>= 10
        channel = data[:, 1] & 0x01
        return np.vstack([timestamps, channel, x, y, polarity]).T.astype(np.float)

    def decodeGaussianAE(self, data):  # TODO: add GAE decoding (the one I am using is sending the radius outside
        pass

    def decodeSkinAE(self, data):
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


    def run(self):
        print('Started running!')
        while(True):
            data = self._reader_port.read()  # blocking call
            ev_type = data.get(0).asString()
            events = np.array(data.get(1).asList().toString().split(' '), dtype=np.uint32)
            events = events.reshape(int(len(events)/2), 2)
            if ev_type == 'AE':
                timestamps, channel, x, y, polarity = self.decodeAddressEvents(events)[0]
            elif ev_type == 'GAE':
                pass
            elif ev_type == 'SAE':
                timestamps, polarity, taxel, cross_base, body_part, side, type, skin = self.decodeSkinAE(events)[0]
            else:
                print('Unrecognised type of event!')



def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='python implementation of yarp RFModule')
    parser.add_argument('--reader_port', type=str, dest='reader_port', help='name of the reading port',
                        default='/module:i')
    #add more parameters here
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    # Read input parametres
    args = parse_args()
    module = RFModule(args.reader_port)
    module.run()