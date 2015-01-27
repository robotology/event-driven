#ifndef EXTRACTION_HPP
#define EXTRACTION_HPP


#include "iCub/emorph/eventBuffer.h"

#include <yarp/os/BufferedPort.h>

#include "extractionThread.hpp"

class extraction:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer> 
{
public:
    extraction(std::string, unsigned int, std::string, unsigned int, unsigned int, double, double, unsigned int);
    ~extraction();
    
    void onRead(emorph::ebuffer::eventBuffer&);
private:
   extractionThread *extracterThread; 
};

#endif //EXTRACTION_HPP

