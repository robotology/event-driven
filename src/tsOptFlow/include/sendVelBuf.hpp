#ifndef SENDVELBUF_HPP
#define SENDVELBUF_HPP

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

#include "VelocityBuffer.h"

#define RATE 5
typedef unsigned int uint;

class sendVelBuf: public yarp::os::RateThread
{
public:
    sendVelBuf(VelocityBuffer*, yarp::os::Semaphore*, yarp::os::BufferedPort<VelocityBuffer>*, uint&);
    ~sendVelBuf();

    void run();
private:
    VelocityBuffer *velBuf;
    yarp::os::BufferedPort<VelocityBuffer>* port;
    yarp::os::Semaphore *mutex;

    bool init;
    uint refts;
    uint acc;
};

#endif //SENDVELBUF_HPP
