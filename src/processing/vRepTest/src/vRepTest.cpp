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

#include "vRepTest.h"

/**********************************************************/
bool vRepTestHandler::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vRepTest")).asString();
    setName(moduleName.c_str());


    /* create the thread and pass pointers to the module parameters */
    reptest.open(moduleName);

    return true ;
}

/**********************************************************/
bool vRepTestHandler::interruptModule()
{
    reptest.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vRepTestHandler::close()
{
    reptest.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vRepTestHandler::updateModule()
{
    return true;
}

/**********************************************************/
double vRepTestHandler::getPeriod()
{
    return 1;
}

/**********************************************************/
vRepTest::vRepTest()
{
    edge.track();
    //here we should initialise the module
    
}
/**********************************************************/
bool vRepTest::open(const std::string &name)
{
    //and open the input port

    this->useCallback();
    this->setStrict();

    std::string portname;

    portname = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(portname);

    portname = "/" + name + "/dump:o";
    dumper.open(portname);

    portname = "/" + name + "/vBottle:o";
    eventsOut.open(portname);

    portname = "/" + name + "/image:o";
    imPort.open(portname);

    return true;
}

/**********************************************************/
void vRepTest::close()
{
    //close ports
    dumper.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void vRepTest::interrupt()
{
    //pass on the interrupt call to everything needed
    dumper.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void vRepTest::onRead(emorph::vBottle &inBottle)
{
    yarp::os::Stamp yts; getEnvelope(yts);

    emorph::vBottle &outBottle = eventsOut.prepare();
    outBottle.clear();

    //create event queue
    emorph::vQueue q = inBottle.getAll();
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        tWindow.addEvent(**qi);
        fWindow.addEvent(**qi);
        lWindow.addEvent(**qi);
        edge.addEventToEdge((*qi)->getAs<emorph::AddressEvent>());


        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(v && v->getX() < 64)
            outBottle.addEvent(**qi);

    }

    //dump modified dataset
    if(eventsOut.getOutputCount()) {
        eventsOut.setEnvelope(yts);
        eventsOut.writeStrict();
    }

    //dump statistics
    if(dumper.getOutputCount()) {
        yarp::os::Bottle &outBottle = dumper.prepare();
        outBottle.addInt(tWindow.getEventCount());
        outBottle.addInt(fWindow.getEventCount());
        outBottle.addInt(lWindow.getEventCount());
        outBottle.addInt(edge.getEventCount());

        dumper.setEnvelope(yts);
        dumper.writeStrict();
    }

    //make debug image
    if(imPort.getOutputCount()) {
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &image = imPort.prepare();
        image.resize(128 * 2 + 15, 128 * 2 + 15);
        image.zero();
        drawDebug(image, tWindow.getTW(), 5, 5);
        drawDebug(image, fWindow.getTW(), 127 + 10, 5);
        drawDebug(image, lWindow.getTW(), 5, 127 + 10);
        drawDebug(image, edge.getSURF(0, 127, 0, 127), 127+10, 127+10);
        imPort.setEnvelope(yts);
        imPort.writeStrict();
    }

}

void vRepTest::drawDebug(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, const emorph::vQueue &q, int xoff, int yoff)
{

    for(int i = 0; i < q.size(); i++) {
        emorph::AddressEvent *v = q[i]->getUnsafe<emorph::AddressEvent>();
        image(v->getY()+yoff, image.width() - 1 - v->getX() - xoff) =
                yarp::sig::PixelBgr(255, 0, 255);
    }

}

