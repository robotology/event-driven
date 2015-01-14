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
//vWindow
/*////////////////////////////////////////////////////////////////////////////*/

int vWindow::getCurrentWindow(vQueue &sample_q)
{
    if(q.empty())
        return 0;

    //critical section
    mutex.wait();

    //do a copy and a clean
    int lowerthesh = q.back()->getStamp() - windowSize;
    int upperthesh = q.back()->getStamp() + windowSize;

    emorph::vQueue::iterator qi;
    for (qi = q.begin(); qi != q.end();) {
        int etime = (*qi)->getStamp();
        if(etime < upperthesh && etime > lowerthesh)
        {
            //copy it into the new q
            vEvent * newcopy = emorph::createEvent((*qi)->getType());
            *newcopy = **qi;
            sample_q.push_back(newcopy);

            //on to the next event
            qi++;
        }
        else
        {
            //clean it (set qi to next event)
            delete (*qi);
            qi = q.erase(qi);
        }
    }

    mutex.post();

    return q.size();

}

void vWindow::addEvent(emorph::vEvent &event)
{
    //make a copy of the event to add to the circular buffer
    vEvent * newcopy = emorph::createEvent(event.getType());
    *newcopy = event;

    mutex.wait();

    //add the event
    q.push_back(newcopy);

    //check if any events need to be removed
    int lifeThreshold = event.getStamp() - windowSize;
    while(true) {

        if(q.front()->getStamp() < lifeThreshold) {
            delete q.front();
            q.pop_front();
        }
        else
            break;
    }
    mutex.post();

}

/*////////////////////////////////////////////////////////////////////////////*/
//vReadAndSplit
/*////////////////////////////////////////////////////////////////////////////*/
///
/// \brief vReadAndSplit::vReadAndSplit
///
vReadAndSplit::vReadAndSplit(int windowsize)
{
    this->windowsize = windowsize;
}

vReadAndSplit::~vReadAndSplit()
{
    std::map<int, vWindow*>::iterator wi;
    for(wi = windows.begin(); wi != windows.end(); wi++) {
        delete wi->second;
    }

    std::map<int, vQueue*>::iterator qi;
    for(qi = snaps.begin(); qi != snaps.end(); qi++) {
        delete qi->second;
    }
}

bool vReadAndSplit::open(const std::string portName)
{
    this->useCallback();

    std::cout << "Opening BufferedPort::vReadAndSplit" << std::endl;
    std::string name = "/" + portName + "/vBottle:i";
    bool r = BufferedPort<emorph::vBottle>::open(name.c_str());

    //r = r && imgWriter.open(std::string(portName + "yarpImage:o").c_str());

    return r;

}

void vReadAndSplit::close()
{
    std::cout << "Closing BufferedPort::vReadAndSplit" << std::endl;
    BufferedPort<emorph::vBottle>::close();
}



void vReadAndSplit::interrupt()
{
   BufferedPort<vBottle>::interrupt();
}


void vReadAndSplit::onRead(emorph::vBottle &incoming)
{
    emorph::vQueue q;
    emorph::vQueue::iterator qi;

    incoming.getAllSorted(q);
    for(qi = q.begin(); qi != q.end(); qi++) {
        int ch = (*qi)->getChannel();
        if(!windows.count(ch)) {
            windows[ch] = new vWindow(windowsize);
        }
        windows[ch]->addEvent(**qi);
    }

}

void vReadAndSplit::snapshotAllWindows()
{
    std::map<int, vWindow*>::iterator wi;
    for(wi = windows.begin(); wi != windows.end(); wi++) {
        if(!snaps.count(wi->first))
                snaps[wi->first] = new vQueue;
        snaps[wi->first]->clear();
        (wi->second)->getCurrentWindow(*(snaps[wi->first]));
    }
}

emorph::vQueue & vReadAndSplit::getSnap(const int channel)
{
    if(!snaps.count(channel))
            snaps[channel] = new vQueue;
    return *(snaps[channel]);

}


/*////////////////////////////////////////////////////////////////////////////*/
//vFramerModule
/*////////////////////////////////////////////////////////////////////////////*/
vFramerModule::~vFramerModule()
{
    for(int i = 0; i < drawers.size(); i++) {
        for(int j = 0; j < drawers[i].size(); j++) {
            delete drawers[i][j];
        }
    }

    for(int i = 0; i < outports.size(); i++) {
        delete outports[i];
    }

}

bool vFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //read in config file


    //set the module name for port naming
    std::string moduleName = rf.find("name").asString();
    if(moduleName == "")
        moduleName = "vFramerModule";
    setName(moduleName.c_str());

    //set the vFrame options (sensor size and eventlife
    int retinaHeight = rf.find("retinaHeight").asInt();
    if(!retinaHeight) {
        std::cerr << "Warning: setting retina size to default values (128x128)."
                     "Ensure this is correct for your sensor" << std::endl;
        retinaHeight = 128;
    }

    int retinaWidth = rf.find("retinaWidth").asInt();
    if(!retinaWidth) {
        retinaWidth = 128;
    }

    double eventWindow = rf.find("eventWindow").asDouble();
    if(eventWindow == 0) eventWindow = 0.5;
    eventWindow = eventWindow * 100000;

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
        std::string outportname = displayList->get(i*3 + 1).asString();
        outports[i]->open("/" + moduleName + "/" + outportname);

        yarp::os::Bottle * drawtypelist = displayList->get(i*3 + 2).asList();
        for(int j = 0; j < drawtypelist->size(); j++) {
            vDraw * newDrawer = createDrawer(drawtypelist->get(j).asString());
            if(newDrawer) {
                newDrawer->setLimits(retinaWidth, retinaHeight);
                drawers[i].push_back(newDrawer);
            }
            else {
                std::cerr << "Could not find draw tag "
                          << drawtypelist->get(j).asString()
                          << ". No drawer created" << std::endl;
            }
            //make a new drawer
            //drawers[i].push_back(/*a new drawer*/);
        }


    }

    //open our event reader given the channel list
    vReader.open(moduleName);

    //set up the frameRate
    period = rf.find("frameRate").asInt();
    if(period == 0) period = 30;
    period = 1.0 / period;


    return true;

}

bool vFramerModule::interruptModule()
{
    vReader.interrupt();
    for(int i = 0; i < outports.size(); i++)
        outports[i]->interrupt();

    return true;
}

bool vFramerModule::close()
{
    vReader.close();
    for(int i = 0; i < outports.size(); i++)
        outports[i]->close();

    return true;
}

bool vFramerModule::respond(const yarp::os::Bottle& command,
                            yarp::os::Bottle& reply)
{
    //add respond messages
    return true;
}

bool vFramerModule::updateModule()
{

    //get a snapshot of current events
    vReader.snapshotAllWindows();


    //for each output image needed
    for(int i = 0; i < channels.size(); i++) {

        //make a new image
        cv::Mat canvas;

        //get the event queue associated with the correct channel
        emorph::vQueue &q = vReader.getSnap(channels[i]);

        //emorph::vQueue q;

        //for each drawer, draw what is needed
        for(int j = 0; j < drawers[i].size(); j++) {
            drawers[i][j]->draw(canvas, q);
        }
        //debug with opencv
        //std::stringstream ss; ss << "Image: " << i;
        //cv::imshow(ss.str(), canvas);
        //cv::waitKey(1);

        //then copy the image to the port and send it on
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &o = outports[i]->prepare();
        o.resize(canvas.cols, canvas.rows);
        cv::Mat publishMat((IplImage *)o.getIplImage(), false);
        cv::flip(canvas, canvas, 0);
        canvas.copyTo(publishMat);
        outports[i]->write();

    }
    return true;

}

double vFramerModule::getPeriod()
{
    return period;
}

} //namespace emorph
