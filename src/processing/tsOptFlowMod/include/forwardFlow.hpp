/*#ifndef FORWARDFLOW_HPP
#define FORWARDFLOW_HPP

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Matrix.h>

#include "VelocityBuffer.h"

#define RATE 10

typedef unsigned int uint;

class forwardFlow:public yarp::os::RateThread
{
public:
    forwardFlow(yarp::sig::Matrix*, yarp::sig::Matrix*, yarp::os::Semaphore*);
    ~forwardFlow();

    void run();
private:
    yarp::sig::Matrix *vxMat;
    yarp::sig::Matrix *vyMat;
    yarp::sig::Matrix cpyVxMat;
    yarp::sig::Matrix cpyVyMat;
    yarp::os::Semaphore *mutex;

    BufferedPort<VelocityBuffer> port;
};

#endif //FORWARDFLOW_HPP
*/
