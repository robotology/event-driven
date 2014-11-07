// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup
 * @ingroup emorph_lib
 *
 * Data transport method for the eMorph project
 *
 * Author: Arren Glover 2014
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __eBottle__
#define __eBottle__


#include <yarp/os/all.h>
#include <iCub/emorph/eCodec.h>

class eBottle : public yarp::os::Bottle {

public:

    //constructors shouldn't change from Bottle
    eBottle() : yarp::os::Bottle() {}

    void addEvent(emorph::ecodec2::eEvent &e);

    void append(eBottle &eb);

    void get(emorph::ecodec2::eEvent &e, emorph::ecodec2::eEventQueue &q);

private:

    //hide all regular functions
    void add();
    void addDict();
    void addDouble();
    void addInt();
    void addList();
    void addString();
    void addVocab();
    void fromString(const yarp::os::ConstString& text);
    void append(const yarp::os::Bottle &alt);
    void copy();
    void copyPortable();



    int getInt();
    yarp::os::ConstString getString();
    double getDouble();
    Bottle* getList();
    yarp::os::Value& get();
    yarp::os::ConstString tostring();
    yarp::os::Value pop();

};


#endif /*__eBottle__*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------
