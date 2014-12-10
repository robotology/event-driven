// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Giorgio Metta
 * Modified 2012/06/05 by Charles Clercq
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __histBuffer__
#define __histBuffer__


#include <yarp/os/all.h>
#include <iostream>
#include <cstring>

namespace emorph
{
namespace ehist
{
/**
 *
 */
class eventHistBuffer : public yarp::os::Portable
{
public:
	eventHistBuffer();
	eventHistBuffer(unsigned int*, unsigned int, unsigned int*, unsigned int);
	~eventHistBuffer();

    eventHistBuffer(const eventHistBuffer&);
    eventHistBuffer& operator=(const eventHistBuffer&);

    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read(yarp::os::ConnectionReader&);

	void set_hist(unsigned int*, unsigned int, unsigned int*, unsigned int);

	void get_hist(unsigned int**, unsigned int&, unsigned int**, unsigned int&);
    unsigned int *get_histp(){return histp;};
    unsigned int *get_histn(){return histn;};
    unsigned int get_histpsz(){return histpsz;};
    unsigned int get_histnsz(){return histnsz;};

private:
	unsigned int* histp;
	unsigned int* histn;
    unsigned int histnsz;
	unsigned int histpsz;
};

}
}

#endif

