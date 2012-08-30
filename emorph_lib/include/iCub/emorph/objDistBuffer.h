// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Charles Clercq
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef OBJDISTBUFFER_H
#define OBJDISTBUFFER_H

#include <yarp/os/all.h>
#include <cstring>

namespace emorph
{
namespace reco
{

class objDistBuffer : public yarp::os::Portable
{
public:
    objDistBuffer();
    objDistBuffer(double*, int&, bool&);
    ~objDistBuffer();

    objDistBuffer& operator=(const objDistBuffer&);
    objDistBuffer(const objDistBuffer&);
    
    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read(yarp::os::ConnectionReader&);

    void setData(double*, int&, bool&);

    double* getBuffer(){return buffer;};
    int getSize(){return szBuffer;};
    bool getEye(){return eye;};
private:
    void cpyObj(const objDistBuffer&);

    double* buffer;
    int szBuffer;
    bool eye;
};

}
}

#endif
