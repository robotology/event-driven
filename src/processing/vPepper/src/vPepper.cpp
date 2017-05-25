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

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find YARP";
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vPepper.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vPepperModule vPepperInstance;
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return vPepperInstance.runModule(rf);
}

/******************************************************************************/
bool vPepperModule::configure(yarp::os::ResourceFinder &rf)
{

    eventManager.initialise(rf.check("name", yarp::os::Value("/vPepper")).asString(),
                            rf.check("height", 240).asInt(),
                            rf.check("width", 304).asInt(),
                            rf.check("spatialSize", yarp::os::Value(1)).asDouble(),
                            rf.check("temporalSize", yarp::os::Value(100000)).asDouble());
    return eventManager.start();

}

bool vPepperModule::close()
{
    eventManager.stop();
    return yarp::os::RFModule::close();
}

bool vPepperModule::updateModule()
{
    return true;
}

double vPepperModule::getPeriod()
{
    return 5.0;
}
/******************************************************************************/

vPepperIO::~vPepperIO()
{
    if(!outPort.isClosed())
        outPort.close();
    if(!inPort.isClosed())
        inPort.close();
}

void vPepperIO::initialise(std::string name, int height, int width,
                           int spatialSize, int temporalSize)
{
    thefilter.initialise(width, height, temporalSize, spatialSize);
    this->name = name;
}

void vPepperIO::run()
{
    yarp::os::Stamp ystamp;

    while(true) {

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inPort.getNextQ(ystamp);
        }
        if(isStopping()) break;

        ev::vBottle &outBottle = outPort.prepare();
        outBottle.clear();
        outPort.setEnvelope(ystamp);

        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            auto v = is_event<AE>(*qi);
            if(thefilter.check(v->x, v->y, v->polarity, v->channel, v->stamp))
                outBottle.addEvent(*qi);
        }

        if(outBottle.size())
            outPort.writeStrict();
        else
            outPort.unprepare();

        inPort.scrapQ();
    }

}

void vPepperIO::onStop()
{
    outPort.close();
    inPort.close();
    inPort.releaseDataLock();
}

bool vPepperIO::threadInit()
{
    if(!outPort.open(name + "/vBottle:o"))
        return false;
    if(!inPort.open(name + "/vBottle:i"))
        return false;
    return true;
}

