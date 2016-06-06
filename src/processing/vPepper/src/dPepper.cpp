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

#include "dPepper.h"

/**********************************************************/
bool dPepperModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vPepper")).asString();
    setName(moduleName.c_str());

    eventManager.setSpatialSize(rf.check("spatialSize",
                                         yarp::os::Value(1)).asDouble());
    eventManager.setTemporalSize(rf.check("temporalSize",
                                          yarp::os::Value(10000)).asDouble());
    eventManager.setResolution(rf.check("height", 128).asInt(),
                               rf.check("width", 128).asInt());
    eventManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool dPepperModule::interruptModule()
{
    eventManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool dPepperModule::close()
{
    eventManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool dPepperModule::updateModule()
{
    return true;
}

/**********************************************************/
double dPepperModule::getPeriod()
{
    return 1.0;
}

/**********************************************************/
dPepperIO::dPepperIO()
{

    //here we should initialise the module
    temporalSize = 10000;
    spatialSize = 1;
    height = 128;
    width = 128;
    leftWindow = emorph::temporalSurface(width, height, temporalSize);
    rightWindow = emorph::temporalSurface(width, height, temporalSize);

}
/**********************************************************/
bool dPepperIO::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);

    return true;
}

/**********************************************************/
void dPepperIO::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
}

/**********************************************************/
void dPepperIO::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/**********************************************************/
void dPepperIO::onRead(emorph::vBottle &bot)
{
    //create event queue
    yarp::os::Stamp yts;
    this->getEnvelope(yts);
    emorph::vQueue q = bot.getAll();
    //create queue iterator
    emorph::vQueue::iterator qi, wi;

    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    outPort.setEnvelope(yts);

    // get the event queue in the vBottle bot
    //bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {

        //leftWindow.addEvent(**qi);
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        if(v->getY() == 1023) continue;

        //keep each channel independently
        emorph::vQueue tw;
        if(v->getChannel()) {
            rightWindow.addEvent(**qi);
            tw = rightWindow.getSurf(v->getX(), v->getY(), spatialSize);
        }
        else {
            leftWindow.addEvent(**qi);
            tw = leftWindow.getSurf(v->getX(), v->getY(), spatialSize);
        }

        bool addit = false;
        for(wi = tw.begin(); wi != tw.end(); wi++) {
            emorph::AddressEvent *pv = (*wi)->getAs<emorph::AddressEvent>();
            if((pv->getX() != v->getX() || pv->getY() != v->getY()) &&
                    pv->getPolarity() == v->getPolarity() &&
                    pv->getChannel() == v->getChannel()) {
                addit = true;
                break;
            }
        }


        if(addit)
            outBottle.addEvent(**qi);

    }
    //send on the processed events
    outPort.write();

}

void dPepperIO::setTemporalSize(double microseconds)
{
    leftWindow.setTemporalSize(microseconds);
    rightWindow.setTemporalSize(microseconds);
}

void dPepperIO::setSpatialSize(double pixelradius)
{
    spatialSize = pixelradius;
}

void dPepperIO::setResolution(int height, int width)
{
    this->height = height;
    this->width = width;
    leftWindow = emorph::temporalSurface(this->width, this->height, temporalSize);
    rightWindow = emorph::temporalSurface(this->width, this->height, temporalSize);

}

//empty line to make gcc happy
