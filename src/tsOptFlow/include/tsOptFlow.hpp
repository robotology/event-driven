#ifndef TSOPTFLOW_HPP
#define TSOPTFLOW_HPP

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdio>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Matrix.h>

#include "eventBuffer.h"

#include "tsOptFlowThread.hpp"
#include "sendVelBuf.hpp"

class tsOptFlow:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer>
{
public:
    tsOptFlow(uint&, uint&, std::string&, uint&, uint&, uint&, double&, uint&, uint&, uint&, double&, double&, int&, uint&, bool&, yarp::os::BufferedPort<VelocityBuffer>*);
    ~tsOptFlow();
    void onRead(emorph::ebuffer::eventBuffer&);
private:
    
    yarp::os::Semaphore *mutex;
    tsOptFlowThread *tsofThreadPos;
    tsOptFlowThread *tsofThreadNeg;
    sendVelBuf *sendvelbuf;
    yarp::sig::Matrix *vxMat;   
    yarp::sig::Matrix *vyMat;

    VelocityBuffer *velBuf;
};

#endif //TSOPTFLOW_HPP
