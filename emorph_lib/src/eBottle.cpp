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

void eBottle::addEvent(emorph::ecodec::eEvent &e) {

    //first append a searchable string
    //yarp::os::Bottle::append(e.getType()());

    //first encode the even
    Bottle coded = e.encode();

    //then push it into the bottle
    yarp::os::Bottle::append(coded);

}

void eBottle::append(eBottle eb) {

    yarp::os::Bottle::append(eb);

}

//emorph::ecodec::eEvent eBottle::pop() {
    //return (emorph::ecodec::eEvent)yarp::os::Bottle::pop();
    //return emorph::ecodec::eEvent();
//}

emorph::ecodec::eEvent& eBottle::getEvent(int index) {
    //get the range of data we are looking for as a Bottle. we have to mask
    //the fact that some events will be of multiple ints and that we
    //have strings for searchable functionality

    //currently decode is performed on a bottle by attempting to convert it
    //to each bottle type and returning true if it is successful

    //surely we can read the index string to know how to convert it

    //how do I deal with the indexing if I don't know how long each
    //event is stored

    emorph::ecodec::eEventQueue evq;
    //emorph::ecodec::eEvent::decode(/*splice the bottle packet in here*/, evq);
    return *(evq.front());
}
