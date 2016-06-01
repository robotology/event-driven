/*
 * Copyright (C) 2010 eMorph Group iCub Facility
 * Authors: Arren Glover
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

#include "iCub/vFramer.h"
#include <sstream>

namespace emorph {

/*////////////////////////////////////////////////////////////////////////////*/
//vReadAndSplit
/*////////////////////////////////////////////////////////////////////////////*/
bool vReadAndSplit::open(const std::string portName, bool strict)
{
    if(strict) this->setStrict();
    this->useCallback();

    std::cout << "Opening BufferedPort::vReadAndSplit" << std::endl;
    return BufferedPort<emorph::vBottle>::open("/" + portName + "/vBottle:i");
}

void vReadAndSplit::onRead(emorph::vBottle &incoming)
{
    this->getEnvelope(yarptime);
    emorph::vQueue q = incoming.getAllSorted();

    safety.lock();
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
        windows[(*qi)->getChannel()].addEvent(**qi);
    safety.unlock();
}

void vReadAndSplit::snapshotAllWindows()
{
    std::map<int, vTempWindow>::iterator wi;

    safety.lock();
    for(wi = windows.begin(); wi != windows.end(); wi++)
        snaps[wi->first] = wi->second.getWindow();
    safety.unlock();
}

const emorph::vQueue& vReadAndSplit::getSnap(const int channel)
{
    return snaps[channel];
}


/*////////////////////////////////////////////////////////////////////////////*/
//vFramerModule
/*////////////////////////////////////////////////////////////////////////////*/
vFramerModule::~vFramerModule()
{
    for(unsigned int i = 0; i < drawers.size(); i++) {
        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            delete drawers[i][j];
        }
    }

    for(unsigned int i = 0; i < outports.size(); i++) {
        delete outports[i];
    }

}

bool vFramerModule::configure(yarp::os::ResourceFinder &rf)
{

    //admin options
    std::string moduleName =
            rf.check("name", yarp::os::Value("vFramer")).asString();
    setName(moduleName.c_str());

    int retinaHeight = rf.check("height", yarp::os::Value(128)).asInt();
    int retinaWidth = rf.check("width", yarp::os::Value(128)).asInt();

    double eventWindow =
            rf.check("eventWindow", yarp::os::Value(0.5)).asDouble();
    eventWindow = eventWindow / vtsHelper::tstosecs();

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    //viewer options
    //set up the default channel list
    yarp::os::Bottle tempDisplayList, *bp;
    tempDisplayList.addInt(0);
    tempDisplayList.addString("Left");
    bp = &(tempDisplayList.addList()); bp->addString("AE");
    tempDisplayList.addInt(1);
    tempDisplayList.addString("Right");
    bp = &(tempDisplayList.addList()); bp->addString("AE");

    //set the output channels
    yarp::os::Bottle * displayList = rf.find("displays").asList();
    if(!displayList)
        displayList = &tempDisplayList;

    std::cout << displayList->toString() << std::endl;

    if(displayList->size() % 3) {
        std::cerr << "Error: display incorrectly configured in provided "
                     "settings file." << std::endl;
        return false;
    }

    int nDisplays = displayList->size() / 3;

    //for each channel open an vFrame and an output port
    channels.resize(nDisplays);
    outports.resize(nDisplays);
    drawers.resize(nDisplays);

    for(int i = 0; i < nDisplays; i++) {

        //extract the channel integer value
        channels[i] = displayList->get(i*3).asInt();

        //extract the portname
        outports[i] = new yarp::os::BufferedPort<
                yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
        //outports[i]->setStrict();
        std::string outportname = displayList->get(i*3 + 1).asString();
        if(!outports[i]->open("/" + moduleName + "/" + outportname))
            return false;

        yarp::os::Bottle * drawtypelist = displayList->get(i*3 + 2).asList();
        for(int j = 0; j < drawtypelist->size(); j++) {
            vDraw * newDrawer = createDrawer(drawtypelist->get(j).asString());
            if(newDrawer) {
                newDrawer->setLimits(retinaWidth, retinaHeight);
                newDrawer->setWindow(eventWindow);
                newDrawer->initialise();
                drawers[i].push_back(newDrawer);
            }
            else {
                std::cerr << "Could not find draw tag "
                          << drawtypelist->get(j).asString()
                          << ". No drawer created" << std::endl;
            }
        }
    }

    //open our event reader given the channel list
    if(!vReader.open(moduleName, strict))
        return false;

    //set up the frameRate
    period = 1.0 / rf.check("frameRate", yarp::os::Value(30)).asInt();

    pyarptime = 0;

    return true;

}

bool vFramerModule::interruptModule()
{
    std::cout << "Interrupting" << std::endl;
    vReader.interrupt();
    for(unsigned int i = 0; i < outports.size(); i++)
        outports[i]->interrupt();
    RFModule::interruptModule();
    std::cout << "Done" << std::endl;
    return true;
}

bool vFramerModule::close()
{
    std::cout << "Closing" << std::endl;
    vReader.close();
    for(unsigned int i = 0; i < outports.size(); i++)
        outports[i]->close();
    RFModule::close();
    std::cout << "Done" << std::endl;

    return true;
}

bool vFramerModule::updateModule()
{

    if(isStopping()) return false;

    yarp::os::Stamp yarptime = vReader.getYarpTime();

    if(yarptime.isValid()) {
        //std::cout << "yarptime valid: " << yarptime.getTime() << std::endl;
        double dt = yarptime.getTime() - pyarptime;
        if(dt < 0)
            //we restarted something from yarpdataplayer
            pyarptime = yarptime.getTime();

        if(yarptime.getTime() - pyarptime < period)
            return true;
        pyarptime = yarptime.getTime();
    }/* else {
        return true;
    }*/

    //get a snapshot of current events
    //std::cout << yarptime.getCount() << std::endl;
    vReader.snapshotAllWindows();

    //for each output image needed
    for(unsigned int i = 0; i < channels.size(); i++) {

        //make a new image
        cv::Mat canvas;

        //get the event queue associated with the correct channel
        const emorph::vQueue &q = vReader.getSnap(channels[i]);

        //for each drawer, draw what is needed
        for(unsigned int j = 0; j < drawers[i].size(); j++) {
            drawers[i][j]->draw(canvas, q);
        }

        //then copy the image to the port and send it on
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &o = outports[i]->prepare();
        o.resize(canvas.cols, canvas.rows);
        cv::Mat publishMat((IplImage *)o.getIplImage(), false);
        cv::flip(canvas, canvas, 0);
        canvas.copyTo(publishMat);
        if(yarptime.isValid()) outports[i]->setEnvelope(yarptime);
        outports[i]->write();
    }

    return true;

}

double vFramerModule::getPeriod()
{
    return 0.3 * period;
}

} //namespace emorph
