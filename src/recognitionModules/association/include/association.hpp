#ifndef ASSOCIATION_HPP
#define ASSOCIATION_HPP

#include "iCub/emorph/eventBuffer.h"
#include <yarp/os/BufferedPort.h>

#include "associationThread.hpp"

class association:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer>
{
public:
    association(std::string, unsigned int, std::string, std::string, unsigned int, unsigned int, double, double, unsigned int, bool, yarp::os::BufferedPort<emorph::ehist::eventHistBuffer>*);
    ~association();

    virtual void onRead(emorph::ebuffer::eventBuffer&);
private:
    associationThread *associaterThread;
};

#endif //ASSOCIATION_HPP
