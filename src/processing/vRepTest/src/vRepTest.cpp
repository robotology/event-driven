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

    std::string vis = rf.check("vis", yarp::os::Value("all")).asString();

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();


    reptest.setVisType(vis);
    //reptest.setTemporalWindow(rf.check("tWin", yarp::os::Value(125000)).asInt());
    reptest.setFixedWindow(rf.check("fWin", yarp::os::Value(1000)).asInt());

    /* create the thread and pass pointers to the module parameters */
    return reptest.open(moduleName, strict);

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
    fWindow.setFixedWindowSize(1000);
    tWindow.setTemporalSize(125000);
    edge.setThickness(1);
    ytime = 0;
    //here we should initialise the module

}
/**********************************************************/
bool vRepTest::open(const std::string &name, bool strict)
{
    //and open the input port

    this->useCallback();
    if(strict) this->setStrict();

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
    if(ytime == 0) ytime = yts.getTime() + 0.033;
    unsigned long unwts = 0;

    //create event queue
    emorph::vQueue q = inBottle.getAll();
    q.sort(true);
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent * ae = (*qi)->getAs<emorph::AddressEvent>();
        if(!ae || ae->getChannel()) continue;
        unwts = unwrapper((*qi)->getStamp());
        tWindow.addEvent(**qi);
        fWindow.addEvent(**qi);
        lWindow.addEvent(**qi);
        edge.addEventToEdge((*qi)->getAs<emorph::AddressEvent>());
        fedge.addEventToEdge((*qi)->getAs<emorph::AddressEvent>());
    }

    //dump modified dataset
    if(eventsOut.getOutputCount() && q.size()) {
        emorph::vBottle &outBottle = eventsOut.prepare();
        outBottle.clear();

        for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
        {
            emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
            if(v && v->getX() < 128)
                outBottle.addEvent(**qi);
        }

        eventsOut.setEnvelope(yts);
        eventsOut.writeStrict();
    }

    //dump statistics
    if(dumper.getOutputCount() && q.size()) {
        yarp::os::Bottle &outBottle = dumper.prepare();
        outBottle.clear();
        outBottle.addInt64(unwts);
        outBottle.addInt(tWindow.getEventCount());
        outBottle.addInt(fWindow.getEventCount());
        outBottle.addInt(lWindow.getEventCount());
        outBottle.addInt(edge.getEventCount());

        dumper.setEnvelope(yts);
        dumper.writeStrict();
    }

    //make debug image
    if(yts.getTime() < ytime - 0.01)
        ytime = yts.getTime();

    if(imPort.getOutputCount() && yts.getTime() > ytime) {
        ytime += 0.01;
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &image = imPort.prepare();

        if(vistype == "all") {
            image.resize(128 * 3 + 20, 128 * 2 + 15);
            image.zero();
            drawDebug(image, tWindow.getSurf(), 5, 5);
            drawDebug(image, fWindow.getSurf(), 5, 127 + 10);
            drawDebug(image, lWindow.getSurf(), 127 + 10, 5);
            drawDebug(image, edge.getSurf(0, 127, 0, 127), 127+10, 127+10);
            drawDebug(image, fedge.getSURF(0, 127, 0, 127), 127+127+15, 127+10);
        } else {
            image.resize(128, 128);
            image.zero();
        }

        if(vistype == "time")
            drawDebug(image, tWindow.getSurf(), 0, 0);
        else if(vistype == "fixed")
            drawDebug(image, fWindow.getSurf(), 0, 0);
        else if(vistype == "life")
            drawDebug(image, lWindow.getSurf(), 0, 0);
        else if(vistype == "edge")
            drawDebug(image, edge.getSurf(0, 127, 0, 127), 0, 0);
        else if(vistype == "fedge")
            drawDebug(image, fedge.getSURF(0, 127, 0, 127), 0, 0);

        imPort.setEnvelope(yts);
        imPort.writeStrict();
    }

}

void vRepTest::drawDebug(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                         const emorph::vQueue &q, int xoff, int yoff)
{

    for(unsigned int i = 0; i < q.size(); i++) {
        emorph::AddressEvent *v = q[i]->getUnsafe<emorph::AddressEvent>();
//        if(q[i]->getAs<emorph::FlowEvent>())
//            image(v->getY()+yoff, image.width() - 1 - v->getX() - xoff) =
//                    yarp::sig::PixelBgr(0, 255, 0);
//        else
            image(v->getY()+xoff, image.height() - 1 - v->getX() - yoff) =
                    yarp::sig::PixelBgr(255, 0, 255);
    }

}

