import yarp
import sys
import numpy as np
import matplotlib.pyplot as plt
import threading


class ExampleModule(yarp.RFModule, threading.Thread):

    def __init__(self):
        yarp.RFModule.__init__(self)
        threading.Thread.__init__(self)
        self.image = np.zeros((240, 304))
        self.image_buf = np.zeros((240, 304))
        self.im = plt.imshow(self.image, cmap='gist_gray_r', vmin=0, vmax=1)
        self.input_port = yarp.BufferedPortBottle()
        self.rpc_port = yarp.RpcServer()
        self.mutex = threading.Lock()

    def configure(self, rf):
        # set the module name used to name ports
        self.setName((rf.check("name", yarp.Value("/exampleModule")).asString()))

        # open io ports
        if not self.input_port.open(self.getName() + "/AE:i"):
            print("Could not open input port")
            return False
        self.input_port.setStrict()

        if not self.rpc_port.open(self.getName() + "/rpc"):
            print("Could not open rpc port")
            return False
        self.attach_rpc_server(self.rpc_port)  # rpc port receives command in the respond method

        # read flags and parameters
        example_flag = rf.check("example_flag") and rf.check("example_flag", yarp.Value(True)).asBool()
        default_value = 0.1
        example_parameter = rf.check("example_parameter", yarp.Value(default_value)).asDouble()

        # do any other set-up required here
        # start the asynchronous and synchronous threads
        self.start()
        return True

    def respond(self, command, reply):
        # Add any command you want to receive from rpc here
        print(command.toString())
        reply.addString('ok')
        return True

    def getPeriod(self):
        return 0.03  # period of synchronous thread, return 0 update module called as fast as it can

    def interruptModule(self):
        # interrupting all the ports
        self.input_port.interrupt()
        self.rpc_port.interrupt()
        return True

    def close(self):
        # closing ports
        self.input_port.close()
        self.rpc_port.close()
        return True

    def updateModule(self):
        # synchronous update called every get period seconds.

        # Put visualization, debug prints, etc... here
        self.mutex.acquire()
        self.im.set_data(self.image)
        self.mutex.release()
        plt.draw()
        plt.pause(1e-3)
        return True

    def run(self):
        # asynchronous thread runs as fast as it can

        while self.is_alive():
            reads = max(1, self.input_port.getPendingReads())  # Make sure that we try to read at least one packet

            if reads > 10:
                print("WARNING ", reads)

            self.image_buf.fill(0.5)

            for i in range(reads):
                bot = self.input_port.read()
                # Data in the bottle is organized as <event_type> (<timestamp 1> <event 1> .... <timestamp n> <event n>)
                vType = bot.get(0).asString()
                event_bottle = np.array(bot.get(1).toString().split(' ')).astype(int).reshape(-1, 2)
                timestamps = event_bottle[:, 0]
                events = event_bottle[:, 1]
                x = events >> 12 & 0xFF
                y = events >> 1 & 0x1FF
                pol = events & 0x01
                self.image_buf[x, y] = pol

            self.mutex.acquire()
            self.image = self.image_buf.copy()  # self.image is a shared resource between threads
            self.mutex.release()


if __name__ == '__main__':
    # Initialise YARP
    yarp.Network.init()
    if not yarp.Network.checkNetwork(2):
        print("Could not find network! Run yarpserver and try again.")
        exit(-1)

    # prepare and configure the resource finder
    rf = yarp.ResourceFinder()
    rf.setVerbose(False)
    rf.setDefaultContext("eventdriven")
    rf.setDefaultConfigFile("exampleModule.ini")
    rf.configure(sys.argv)

    # create the module
    module = ExampleModule()
    module.runModule(rf)
