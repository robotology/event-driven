#ifndef READRAW_H
#define READRAW_H

#include <iostream>
#include <cstring>

#include "eventBuffer.h"
//#include "synapse.h"
//#include "eLucasKanade.h"

#include "unmask.h"
//#include "config.h"

#include <yarp/os/Network.h>

//#define _DEBUG
class readRaw:public yarp::os::BufferedPort<eventBuffer>
{
public:
    readRaw();
    ~readRaw();
    virtual void onRead(eventBuffer&);

private:
    unmask objUnmask;
//    synapse objSynapse;
//    eLucasKanade objFlow;
};

#endif //READRAW_H
