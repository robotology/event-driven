// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Charles Clercq
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "iCub/emorph/eventHistBuffer.h"
using namespace std;
using namespace yarp::os;

namespace emorph
{
namespace ehist
{

eventHistBuffer::eventHistBuffer()
{
	histp=NULL;
	histn=NULL;
    histnsz=0;
	histpsz=0;
}

eventHistBuffer::eventHistBuffer(unsigned int* i_hp, unsigned int i_szp, unsigned int* i_hn, unsigned int i_szn)
{
	histp = new unsigned int[4*i_szp];
    memset(histp, 0, 4*i_szp*sizeof(unsigned int));
	memcpy(histp, i_hp, 4*i_szp*sizeof(unsigned int));
	histpsz = i_szp;

	histn = new unsigned int[4*i_szn];
    memset(histn, 0, 4*i_szn*sizeof(unsigned int));
	memcpy(histn, i_hn, 4*i_szn*sizeof(unsigned int));
	histnsz = i_szn;
}

eventHistBuffer::eventHistBuffer(const eventHistBuffer& buffer) {
    histpsz=buffer.histpsz;
    delete[] histp;
	histp  = new unsigned int[4*histpsz];
    memset(histp, 0, 4*histpsz*sizeof(unsigned int));
    if (histpsz > 0)
        memcpy(histp, buffer.histp, 4*histpsz*sizeof(unsigned int));

    histnsz=buffer.histnsz;
    delete[] histn;
	histn  = new unsigned int[4*histnsz];
    memset(histn, 0, 4*histnsz*sizeof(unsigned int));
    if (histnsz > 0)
        memcpy(histn, buffer.histn, 4*histnsz*sizeof(unsigned int));
}

eventHistBuffer::~eventHistBuffer()
{
	delete[] histp;
	delete[] histn;
}

eventHistBuffer& eventHistBuffer::operator=(const eventHistBuffer& buffer) {
    if(this!=&buffer)
    {
        histpsz=buffer.histpsz;
        delete[] histp;
        histp  = new unsigned int[4*histpsz];
        memset(histp, 0, 4*histpsz*sizeof(unsigned int));
        if (histpsz > 0)
            memcpy(histp, buffer.histp, 4*histpsz*sizeof(unsigned int));

        histnsz=buffer.histnsz;
        delete[] histn;
        histn  = new unsigned int[4*histnsz];
        memset(histn, 0, 4*histnsz*sizeof(unsigned int));
        if (histnsz > 0)
            memcpy(histn, buffer.histn, 4*histnsz*sizeof(unsigned int));
    }
    return *this;
}

void eventHistBuffer::set_hist(unsigned int *i_histp, unsigned int i_szp, unsigned int *i_histn, unsigned int i_szn)
{
    delete[] histp;
    histp=new unsigned int[4*i_szp];
    memset(histp, 0, 4*i_szp*sizeof(unsigned int));
	memcpy(histp, i_histp, 4*i_szp*sizeof(unsigned int));
	histpsz = i_szp;

    delete[] histn;
    histn=new unsigned int[4*i_szn];
    memset(histn, 0, 4*i_szn*sizeof(unsigned int));
	memcpy(histn, i_histn, 4*i_szn*sizeof(unsigned int));
	histnsz = i_szn;
}


void eventHistBuffer::get_hist(unsigned int** o_histp, unsigned int& o_szp, unsigned int** o_histn, unsigned int& o_szn)
{
    *o_histp=new unsigned int[4*histpsz];
    memcpy(*o_histp, histp, 4*histpsz*sizeof(unsigned int));
    *o_histn=new unsigned int[4*histnsz];
    memcpy(*o_histn, histn, 4*histnsz*sizeof(unsigned int));

    o_szp=histpsz;
    o_szn=histnsz;
}

bool eventHistBuffer::write(yarp::os::ConnectionWriter& connection)
{
	connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
	connection.appendInt(4);        // four elements
	connection.appendInt(histpsz);
	connection.appendInt(histnsz);
	connection.appendBlock((char*)histp, 4*histpsz*sizeof(unsigned int));
	connection.appendBlock((char*)histn, 4*histnsz*sizeof(unsigned int));
	connection.convertTextMode();   // if connection is text-mode, convert!
	return true;
}

bool eventHistBuffer::read(yarp::os::ConnectionReader& connection)
{
	connection.convertTextMode();   // if connection is text-mode, convert!
	int tag = connection.expectInt();
	if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
		return false;
	int ct = connection.expectInt();
	if (ct!=4)
		return false;
	histpsz = connection.expectInt();
	histnsz = connection.expectInt();
    if(histp!=NULL)
        delete[] histp;
    histp=new unsigned int[4*histpsz];
    if(histn!=NULL)
        delete[] histn;
    histn=new unsigned int[4*histnsz];
	connection.expectBlock((char*)histp, 4*histpsz*sizeof(unsigned int));
	connection.expectBlock((char*)histn, 4*histnsz*sizeof(unsigned int));
	return true;
}

}
}
