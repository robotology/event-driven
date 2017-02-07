/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "vPepper.h"

/**********************************************************/
bool vPepperModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vPepper")).asString();
    setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    eventManager.initialise(rf.check("height", 240).asInt(),
                            rf.check("width", 304).asInt(),
                            rf.check("spatialSize", yarp::os::Value(1)).asDouble(),
                            rf.check("temporalSize", yarp::os::Value(10000)).asDouble());

    eventManager.open(moduleName, strict);

    return true ;
}

/**********************************************************/
bool vPepperModule::interruptModule()
{
    eventManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vPepperModule::close()
{
    eventManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vPepperModule::updateModule()
{
    return true;
}

/**********************************************************/
double vPepperModule::getPeriod()
{
    return 1.0;
}

/**********************************************************/
vPepperIO::vPepperIO()
{

    //here we should initialise the module
    temporalSize = 10000;
    spatialSize = 1;
    res.height = 128;
    res.width = 128;
    strict = false;

}

void vPepperIO::initialise(int height, int width, int spatialSize, int temporalSize)
{
    res.height = height;
    res.width = width;
    thefilter.initialise(res.width, res.height, temporalSize, spatialSize);

}

/**********************************************************/
bool vPepperIO::open(const std::string &name, bool strict)
{
    //and open the input port

    this->useCallback();
    this->strict = strict;
    if(strict) {
        std::cout << "Using STRICT communication" << std::endl;
        this->setStrict();
    } else {
        std::cout << "NOT using strict communication" << std::endl;
    }

    if(!yarp::os::BufferedPort<ev::vBottle>::open("/" + name + "/vBottle:i"))
        return false;
    if(!outPort.open("/" + name + "/vBottle:o"))
        return false;

    return true;
}

/**********************************************************/
void vPepperIO::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();
}

/**********************************************************/
void vPepperIO::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

/**********************************************************/
void vPepperIO::onRead(ev::vBottle &bot)
{
    //create event queue
    yarp::os::Stamp yts;
    this->getEnvelope(yts);
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    outPort.setEnvelope(yts);

    ev::vQueue q = bot.getAll();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        event<AddressEvent> v = getas<AddressEvent>(*qi);
        if(!v) continue;
        if(thefilter.check(v->getX(), v->getY(), v->getPolarity(), v->getChannel(), v->getStamp())) {
            outBottle.addEvent(v);
        }
    }

    //send on the processed events
    if(strict)
        outPort.writeStrict();
    else
        outPort.write();

}

//empty line to make gcc happy
