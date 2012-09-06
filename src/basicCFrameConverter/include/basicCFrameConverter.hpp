#ifndef C_CARTESIAN_FRAME_CONVERTER
#define C_CARTESIAN_FRAME_CONVERTER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>

#include "convert.hpp"

//#include "sending_buffer.hpp"
//#include "config.hpp"

#include "iCub/emorph/eventBuffer.h"

#include <yarp/os/BufferedPort.h>

//#define _DEBUG

class basicCFrameConverter:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer>
{
public:
    basicCFrameConverter(unsigned int, unsigned int, unsigned int, std::string, unsigned int, unsigned int);
    ~basicCFrameConverter();
    void onRead(emorph::ebuffer::eventBuffer&);

private:
    convert converter;
};

#endif //C_CARTESIAN_FRAME_CONVERTER
