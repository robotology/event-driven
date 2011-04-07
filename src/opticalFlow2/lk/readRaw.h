#ifndef __READRAW_H__
#define __READRAW_H__

#include <yarp/os/BufferedPort.h>

#include "unmask.h"
#include "eventBuffer.h"

class readRaw : public yarp::os::BufferedPort<eventBuffer>
{
public:
    readRaw(int channel) : objUnmask(channel)
    {
    }
    ~readRaw(){}
    
    virtual void onRead(eventBuffer& eb)
    {
        objUnmask.unmaskData((unsigned char*)(eb.get_packet()),eb.get_sizeOfPacket(),4);
    }

private:
    Unmask objUnmask;
};
#endif