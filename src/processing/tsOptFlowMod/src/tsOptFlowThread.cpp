/*
 * @file tsOptFlowThread.cpp
 * @brief Implementation of the tsOptFlowThread (see header file).
 */

#include "tsOptFlowThread.hpp"

#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/eventCodec.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace emorph::ecodec;
using namespace emorph::eunmask;
using namespace emorph::ebuffer;
using namespace std;

tsOptFlowThread::tsOptFlowThread():Thread()
{
   /* alpha=a;
    threshold=th;
    tauD=td;
    tsVal=tsval;
    height=h;
    width=w;

    accumulation=acc;
    vxMean=new double[h*w]; memset(vxMean, 0, _h*_w*sizeof(double));
    vyMean=new double[h*w]; memset(vyMean, 0, _h*_w*sizeof(double));
    ivxyNData=new uint[h*w]; memset(ivxyNData, 0, _h*_w*sizeof(uint));
*/
}

tsOptFlowThread::~tsOptFlowThread()
{


}

bool tsOptFlowThread::threadInit()
{
    return true;
}

void tsOptFlowThread::onRead(eventBuffer &_buf)
{

}

void tsOptFlowThread::onStop()
{

}

void tsOptFlowThread::threadRelease()
{

}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
