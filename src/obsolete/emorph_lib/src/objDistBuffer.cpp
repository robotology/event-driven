// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Charles Clercq
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iCub/emorph/objDistBuffer.h>
namespace emorph
{
namespace reco
{

objDistBuffer::objDistBuffer()
{
    buffer=NULL;
    szBuffer=0;
    eye=false;
}

objDistBuffer::objDistBuffer(double *_buf, int &_sz, bool &_eye)
{
    szBuffer=_sz;
    buffer=new double[_sz];
    memcpy(buffer, _buf, _sz*sizeof(double));
    eye=_eye;
}

objDistBuffer::~objDistBuffer()
{
    delete[] buffer;
}

objDistBuffer::objDistBuffer(const objDistBuffer &_obj)
{
    cpyObj(_obj);
}

objDistBuffer& objDistBuffer::operator=(const objDistBuffer &_obj)
{
    cpyObj(_obj);
    return *this;
}

void objDistBuffer::cpyObj(const objDistBuffer &_obj)
{
    if(&_obj!=this)
    {
        delete[] buffer;
        buffer=new double[_obj.szBuffer];
        memcpy(buffer, _obj.buffer, _obj.szBuffer*sizeof(double));
        //buffer=_obj.buffer;
        szBuffer=_obj.szBuffer;
        eye=_obj.eye;
    }
}

void objDistBuffer::setData(double *_buf, int &_sz, bool &_eye)
{
    delete[] buffer;
    szBuffer=_sz;
    buffer=new double[_sz];
    memcpy(buffer, _buf, _sz*sizeof(double));
    eye=_eye;
}

bool objDistBuffer::write(yarp::os::ConnectionWriter& connection)
{
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
    connection.appendInt(3);
    connection.appendInt(eye);
    connection.appendInt(szBuffer);
    connection.appendBlock(reinterpret_cast<char*>(buffer), szBuffer*sizeof(double));
    connection.convertTextMode();   // if connection is text-mode, convert!
    return true;

}

bool objDistBuffer::read(yarp::os::ConnectionReader& connection)
{
    connection.convertTextMode();   // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
        return false;
    int ct = connection.expectInt();
    if (ct!=3)
        return false;
    eye = connection.expectInt();
    szBuffer = connection.expectInt();
    delete[] buffer;
    buffer=new double[szBuffer];
    connection.expectBlock(reinterpret_cast<char*>(buffer), szBuffer*sizeof(double));
    return true;
}

}
}
