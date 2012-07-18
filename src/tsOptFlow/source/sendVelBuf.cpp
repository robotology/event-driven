#include "sendVelBuf.hpp"

using namespace yarp::os;
sendVelBuf::sendVelBuf(VelocityBuffer* _velb, Semaphore *_sem, BufferedPort<VelocityBuffer> *_port, uint &_acc)
:RateThread(RATE), acc(_acc)
{
    velBuf=_velb;
    mutex=_sem;
    port=_port;

    init=true;
}

sendVelBuf::~sendVelBuf()
{
}

void sendVelBuf::run()
{
    //std::cout << "Mutex free?" << std::endl;
    mutex->wait();
    //std::cout << "\tMutex took" << std::endl;
    if(!velBuf->isEmpty())
    {
        if(init)
        {
            refts=velBuf->getTs(0);
            init=false;
        }
        else if( (refts+acc)<velBuf->getTs(velBuf->getSize()-1))
        {
            refts=velBuf->getTs(0);
            VelocityBuffer& tmp = port->prepare();
            tmp=*velBuf;
            port->write();
            velBuf->emptyBuffer();
        }
    }
    mutex->post();
   // std::cout << "Mutex freed" << std::endl;
}
