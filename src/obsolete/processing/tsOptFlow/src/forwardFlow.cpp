#include "forwardFlow.hpp"

using namespace yarp::os;
using namespace yarp::sig;
forwardFlow::forwardFlow(Matrix *_vxMat, Matrix *_vyMat, Semaphore *_mutex)
:RateThread(RATE)
{
    vxMat=_vxMat;
    vyMat=_vyMat;

    mutex=_mutex;

    port.open("/tsOptFlow:o");
}

forwardFlow::~forwardFlow()
{
    port.close();
}

void forwardFlow::run()
{
    mutex->check();
    cpyVxMat=*vxMat;
    cpyVyMat=*vyMat;
    mutex->post();

    VelocityBuffer velBuf;
    for(short i=0; i<cpyVxMat.rows(); ++i)
        for(short ii=0; ii<cpyVxMat.cols(); ++ii)
        {
            if( cpyVxMat(i, ii)!=0 || cpyVyMat(i, ii)!=0 )
                velBuf.addData(i, ii, cpyVxMat(i, ii), cpyVyMat(i, ii), 0);
        }
}

