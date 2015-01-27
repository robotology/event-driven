/*
 * @file tsOptFlowThread.pp
 * @brief Definition of a thread that extracts the optical flow
*/
#ifndef TSOPTFLOWTHREAD_HPP
#define TSOPTFLOWTHREAD_HPP

#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventCodec.h>
#include <iCub/emorph/eventBuffer.h>
#include "iCub/emorph/eventUnmask.h"
#include "iCub/emorph/eventUnmaskDVS128.h"
#include "iCub/emorph/eventUnmaskICUB.h"
#include "iCub/emorph/eventUnmaskICUBcircBuf.h"

class tsOptFlowThread : public yarp::os::Thread
{
    /*int refts;    //time reference
    uint timestamp;    //timestamp of the event
    int iBinEvts;    //index of the matrix BinEvts
    yarp::sig::Matrix binEvts;    //matrix containing the events
    double alpha;
    double *vxMean;
    double *vyMean;
    int *ivxyNData;
    uint addrx;
    uint addry;
    int polarity;
    uint eye;
    emorph::eunmask::eventUnmask *unmasker;*/

public:
   /*default constructor*/
    tsOptFlowThread();

    /*destructor*/
    ~tsOptFlowThread();

    /*function that initialises the thread*/
    bool threadInit();

    /*function called when the thread is stopped*/
    void threadRelease();

    /*function that reads event buffer*/
    void onRead(emorph::ebuffer::eventBuffer &);

    /*function called when the module is poked with an interrupt command*/
    void interrupt();

    /*function called when the thread is stopped*/
    void onStop();

};

#endif //TSOPTFLOWTHREAD_HPP

//----- end-of-file --- ( next line intentionally left blank ) ------------------
