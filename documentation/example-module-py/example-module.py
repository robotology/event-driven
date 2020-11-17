import yarp
import sys

#inizializzato una volta in configure o init

# online run in update


class exampleModule(yarp.RFModule):

    def __init__(self):
        yarp.RFModule.__init__(self)

    def configure(self, rf):

        self.input_port = yarp.Port()
        self.rpc_port = yarp.RpcServer()

        # set the module name used to name ports
        self.setName((rf.check("name", yarp.Value("/example-module")).asString()))

        # open io ports
        if not self.input_port.open(self.getName() + "/AE:i"):
            print("Could not open input port")
            return False
        if not self.rpc_port.open(self.getName() + "/rpc"):
            print("Could not open rpc port")
            return False
        self.attach_rpc_server(self.rpc_port)  # rpc addressed to a response method

        # read flags and parameters
        example_flag = rf.check("example_flag") and rf.check("example_flag", yarp.Value(True)).asBool()
        default_value = 0.1
        example_parameter = rf.check("example_parameter",
                                     yarp.Value(default_value)).asDouble()

        # do any other set-up required here
        # start the asynchronous and synchronous threads
        return True

    def respond(self, command, reply):
        print(command.toString())
        reply.addString('ok')
        return True


    def getPeriod(self):
        return 3.0  # period of synchrnous thread, return 0 update module called as fast as it can

    def interruptModule(self):
        # interrupting all the ports
        self.input_port.interrupt()
        self.rpc_port.interrupt()
        return True

    def close(self):
        # closing ports
        self.input_port.close()
        self.rpc_port.close()

    def updateModule(self):
        # synchronous update called every get period seconds.

        # THE CORE OF YOUR COMPUTATION SHOULD BE HERE
        print('updating ...')
        return True


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
    rf.setDefaultConfigFile("example-module.ini")
    rf.configure(sys.argv)

    # create the module
    instance = exampleModule()
    instance.runModule(rf)

