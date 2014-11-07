// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup
 * @ingroup emorph_lib
 *
 * Data transport method for the eMorph project using the standard Bottle
 * format tailored for eEvents
 *
 * Author: Arren Glover 2014
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iCub/emorph/eBottle.h>

void eBottle::addEvent(emorph::ecodec2::eEvent &e) {

    //first append a searchable string
    //yarp::os::Bottle::addString(e.getType());
    yarp::os::Bottle * b = yarp::os::Bottle::find(e.getType()).asList();

    if(!b) {
        yarp::os::Bottle::addString(e.getType());
        b = &(yarp::os::Bottle::addList());
    }

    //add the coded event to the end of the bottle
    b->append(e.encode());

}

void eBottle::append(eBottle &eb) {

    yarp::os::Bottle *alt = &eb;
    yarp::os::Bottle::append(*alt);

}

void eBottle::get(emorph::ecodec2::eEvent &e, emorph::ecodec2::eEventQueue &q)
{

    Bottle * b = find(e.getType()).asList();

    int pos = 0;
    while(pos < b->size())
        q.push_back(e.decode(*b, pos));

}

