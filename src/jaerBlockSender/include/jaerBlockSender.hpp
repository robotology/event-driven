#ifndef JAERBLOCKSENDER_HPP
#define JAERBLOCKSENDER_HPP

#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include "iCub/emorph/eventBuffer.h"
#include "iCub/emorph/eventUnmask.h"
#include "iCub/emorph/eventUnmaskDVS128.h"

class jaerBlockSender
{
public:
    jaerBlockSender();
    ~jaerBlockSender();

    int load(std::string);
    void send(int);
    int sendBlocks(uint, uint, uint);
    void prepareData(uint);
private:
    int index;
    int bodySize;
    int jaerSizeType;
    int currentEvent;

    char* buffer;

    std::vector<uint> indexes;
    yarp::os::BufferedPort<emorph::ebuffer::eventBuffer> outputPort;
};

#endif //JAERBLOCKSENDER_HPP
