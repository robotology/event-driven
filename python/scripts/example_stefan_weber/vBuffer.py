#!/usr/bin/env python3

# Author: Stefan Weber
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

import yarp
import numpy as np
import event_driven
import queue


class VBottleBuffer(yarp.PortReader):
    def __init__(self, timestep, portname):
        super().__init__()
        self.storedEvents = Buffer(timestep)
        self.timeFrameQueue = queue.Queue()
        self.portname = portname

    def addEvents(self, data):
        out = self.storedEvents.add_data(data)
        # print(data[-1][1], data[0][1])
        if out is not None:
            self.timeFrameQueue.put(out)

    def __enter__(self):
        self.pi = yarp.Port()
        self.pi.setReader(self)
        self.pi.open(self.portname);

    def __exit__(self, exc_type, exc_value, traceback):
        yarp.Network.fini();

    def read(self, connection):
        # print('entered read')
        if not (connection.isValid()):
            print("Connection shutting down")
            return False
        # binp = yarp.Bottle()
        binp = event_driven.vBottle()
        ok = binp.read(connection)
        if not (ok):
            print("Failed to read input")
            return False
        data = event_driven.getData(binp)
        self.addEvents(data)
        return True


class Buffer:
    def __init__(self, timestep):
        self.z = []
        self.current = None
        self.timestep = timestep

    def add_data(self, new_data):
        if self.current is None:
            self.current = new_data[0, 1] // self.timestep
        last = new_data[-1, 1] // self.timestep
        self.z.append(new_data)
        if self.current != last:
            out = np.concatenate([x[np.where((
                x[:, 1] // self.timestep == self.current))] for x in self.z])
            self.z = self.z[-1:]
            self.current = last
            return out
        return None


if __name__ == '__main__':
    yarp.Network.init()
    bottleBuffer = VBottleBuffer(100000, "/buffer:i")
    with bottleBuffer as buf:
        while True:
            data = bottleBuffer.timeFrameQueue.get()
            print(data.shape, data[-1][1], data[0][1])
