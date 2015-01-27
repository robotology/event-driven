/*
 * @file tsOptFlow.cpp
 * @brief Implementation of the tsOptFlow (see header file).
 */

#include "tsOptFlow.hpp"
#include "tsOptFlowThread.hpp"
#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventCodec.h>
#include "VelocityBuffer.h"
#include "sendVelBuf.hpp"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace emorph::ecodec;
using namespace emorph::ebuffer;


bool tsOptFlow::configure(ResourceFinder &rf)
{
    printf("initialization of the main thread \n");

    moduleName            = rf.check("name",
                           Value("/tsOptFlow"),
                           "module name (string)").asString();

    setName(moduleName.c_str());

    /* open rpc port */
    handlerPortName =  "";
    handlerPortName += getName();

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    printf("attaching handler port \n");
    attach(handlerPort);

    /* open input port */
    printf("opening input port \n");
    inPortName = "";
    inPortName += getName();
    inPortName += "/evts:i";

    if (!inPort.open(inPortName.c_str())) {
        cout << getName() << ": Unable to open input port " << inPortName << endl;
        return false;
    }

    /* open output port */
    printf("opening output port \n");
    outPortName = "";
    outPortName += getName();
    outPortName += "/flow:o";

    if (!outPort.open(outPortName.c_str()))
    {
            cerr << getName() << ": Unable to open output port " << outPortName << endl;
            return false;
    }

    /* create the thread and pass pointers to the module parameters */
    printf("starting tsOptFlowThread \n");
    //tsofThread=new tsOptFlowThread();

    //printf("starting sending output \n");
    //sendvelbuf=new sendVelBuf();

    /* start the threads */
    //tsofThread->open();
    //sendvelbuf->open();

    return true ;
}

bool tsOptFlow::interruptModule()
{
    handlerPort.interrupt();
    inPort.interrupt();
    outPort.interrupt();
    //tsofThread->interrupt();
    //sendvelbuf->interrupt();
    return true;
}

bool tsOptFlow::close()
{
    handlerPort.close();
    inPort.close();
    outPort.close();
    printf("Ports closed \n");

    /* stop the thread */
    printf("starting the shutdown procedure \n");
    //tsofThread->close();
    //sendvelbuf->close();

    printf("deleting threads \n");
    //delete tsofThread;
    //delete sendvelbuf;
    printf("done deleting threads \n");
    return true;
}

/* Called periodically every getPeriod() seconds */
bool tsOptFlow::updateModule()
{
    return true;
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
