/*
 * @file tsOptFlow.h
 * @brief A module that reads eventBuffer from a yarp port and computes optical flow
 */
#ifndef TSOPTFLOW_HPP
#define TSOPTFLOW_HPP

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>

//within project includes
#include "tsOptFlowThread.hpp"
#include "sendVelBuf.hpp"
#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventCodec.h>
#include <iCub/emorph/VelocityBuffer.h>

/*class vtsOptFlow:public yarp::os::BufferedPort<vBottle>
{
    onRead(vBottle);
    tsOptFlowThread;
}
*/

class tsOptFlow:public yarp::os::RFModule {

    /* module parameters*/
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string handlerPortName;                //name of the handler port (comunication with respond function)
    yarp::os::RpcServer handlerPort;            //a port to handle messages
    std::string inPortName;    //name of the input port
    std::string outPortName;    //name of the output port
    yarp::os::BufferedPort<eventBottle> inPort;    //a port to receive events
    yarp::os::BufferedPort<VelocityBuffer> outPort;    //a port to send velocity buffers

    /* pointer to threads*/
    tsOptFlowThread* tsofThread;                //tsofThread for processing events and computing optical flow
    sendVelBuf *sendvelbuf;                     //a thread to send Velocity Buffer to output port

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool updateModule();
};
#endif //TSOPTFLOW_HPP

//----- end-of-file --- ( next line intentionally left blank ) ------------------
