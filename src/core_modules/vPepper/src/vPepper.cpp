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

    eventManager.setSpatialSize(rf.check("spatialSize",
                                         yarp::os::Value(1)).asDouble());
    eventManager.setTemporalSize(rf.check("temporalSize",
                                          yarp::os::Value(10000)).asDouble());
    eventManager.setResolution(rf.check("height", 128).asInt(),
                               rf.check("width", 128).asInt());
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
    height = 128;
    width = 128;
    leftWindow = ev::temporalSurface(width, height, temporalSize);
    rightWindow = ev::temporalSurface(width, height, temporalSize);
    strict = false;

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

    yarp::os::BufferedPort<ev::vBottle>::open("/" + name + "/vBottle:i");
    outPort.open("/" + name + "/vBottle:o");

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

    //return;

    ev::vQueue q = bot.getAll();
    //create queue iterator
    ev::vQueue::iterator qi, wi;

    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    outPort.setEnvelope(yts);

    // get the event queue in the vBottle bot
    //bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {

        //leftWindow.addEvent(**qi);
        ev::event<ev::AddressEvent> v = ev::getas<ev::AddressEvent>(*qi);
        if(!v) continue;
        if(v->getY() == 1023) continue; //put in for USB ATIS

        //keep each channel independently
        ev::vQueue tw;
        if(v->getChannel()) {
            rightWindow.addEvent(*qi);
            tw = rightWindow.getSurf(v->getX(), v->getY(), spatialSize);
        }
        else {
            leftWindow.addEvent(*qi);
            tw = leftWindow.getSurf(v->getX(), v->getY(), spatialSize);
        }

        bool addit = false;
        for(wi = tw.begin(); wi != tw.end(); wi++) {
            ev::event<ev::AddressEvent> pv = ev::getas<ev::AddressEvent>(*wi);
            if((pv->getX() != v->getX() || pv->getY() != v->getY()) &&
                    pv->getPolarity() == v->getPolarity() &&
                    pv->getChannel() == v->getChannel()) {
                addit = true;
                break;
            }
        }


        if(addit)
            outBottle.addEvent(*qi);

    }
    //send on the processed events
    if(strict)
        outPort.writeStrict();
    else
        outPort.write();

}

void vPepperIO::setTemporalSize(double microseconds)
{
    temporalSize = microseconds;
    leftWindow.setTemporalSize(microseconds);
    rightWindow.setTemporalSize(microseconds);
}

void vPepperIO::setSpatialSize(double pixelradius)
{
    spatialSize = pixelradius;
}

void vPepperIO::setResolution(int height, int width)
{
    this->height = height;
    this->width = width;
    leftWindow = ev::temporalSurface(this->width, this->height, temporalSize);
    rightWindow = ev::temporalSurface(this->width, this->height, temporalSize);

}

//empty line to make gcc happy
