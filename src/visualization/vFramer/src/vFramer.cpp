/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Arren Glover (@itt.it)
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
//eFrame
/*////////////////////////////////////////////////////////////////////////////*/
///
/// \brief eFrame::eFrame
/// \param retinaWidth
/// \param retinaHeight
/// \param windowWidth
/// \param windowHeight
///
///
eFrame::eFrame(int channel, int retinaWidth, int retinaHeight)
{
    this->channel = channel;
    this->retinaWidth = retinaWidth;
    this->retinaHeight = retinaHeight;

    eventLife = 50000; //default 0.5 seconds
}

void eFrame::setEventLife(int eventLife)
{
    this->eventLife = eventLife;
}


///
/// \brief eFrame::publish
/// \return
///
void eFrame::publish(cv::Mat &imageOnThePort, double seconds)
{

    //critical section
    mutex.wait();

    //do a quick clean of the q
    if(!q.empty()) {
        int ctime = q.back()->getStamp();
        emorph::eEventQueue::iterator qi = q.begin();
        while(qi != q.end()) {
            int etime = (*qi)->getStamp();
            if(etime < ctime - eventLife || etime > ctime + eventLife) {
                delete (*qi);
                qi = q.erase(qi);
            }
            else
            {
                qi++;
            }
        }
    }

    //draw the q
    cv::Mat canvas = draw(q);
    mutex.post();


    //we can't gaurantee the output of the canvas so we have to do some
    //image conversion (channel count and image size)
    cv::Mat correctChannels(canvas.size(), CV_8UC3);
    if(canvas.type() == CV_8U) {
        cv::cvtColor(canvas, correctChannels, CV_GRAY2BGR);
    } else {
        correctChannels = canvas; //shallow copy
    }

    cv::flip(correctChannels, correctChannels, 0);

    cv::resize(correctChannels, imageOnThePort,
               imageOnThePort.size(), 0, 0, cv::INTER_NEAREST);


}

void eFrame::addEvent(emorph::eEvent &event)
{
    //sketchy hack to remove the unknown artefacts
    //this shouldn't be needed when the artefacts go!
    if(event.getStamp() == 4287744)
        return;


    //make a copy of the event to add to the circular buffer
    eEvent * newcopy = emorph::createEvent(event.getType());
    *newcopy = event;

    mutex.wait();

    //add the event
    q.push_back(newcopy);

    //check if any events need to be removed
    while(true) {

        int lifeThreshold = event.getStamp() - eventLife;

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
//eAddressFrame
/*////////////////////////////////////////////////////////////////////////////*/

///
/// \brief eAddressFrame::addEvent
/// \param event
///
cv::Mat eAddressFrame::draw(eEventQueue &eSet)
{
    int mass = 180;
    //std::cout << "Drawing " << eSet.size() << std::endl;
    emorph::eEventQueue::iterator qi;
    cv::Mat canvas(getRetinaWidth(), getRetinaHeight(), CV_8UC3);
    canvas.setTo(128);

    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(aep) {
            cv::Vec3b cpc = canvas.at<cv::Vec3b>(aep->getX(), aep->getY());

            if(aep->getPolarity())
            {
                if(cpc.val[1] > 255 - mass) cpc.val[1] = 255;
                else cpc.val[1] += mass;
            }
            else
            {
                if(cpc.val[2] > 255 - mass) cpc.val[2] = 255;
                else cpc.val[2] += mass;
            }

            canvas.at<cv::Vec3b>(aep->getX(), aep->getY()) = cpc;
        }

    }

    return canvas;

}

/*////////////////////////////////////////////////////////////////////////////*/
//eReadAndSplit
/*////////////////////////////////////////////////////////////////////////////*/
///
/// \brief eReadAndSplit::eReadAndSplit
///
eReadAndSplit::eReadAndSplit(const std::string &moduleName)
{

    portName = moduleName;

    this->eframes = 0;

}

bool eReadAndSplit::open()
{
    this->useCallback();

    std::cout << "Opening BufferedPort::eReadAndSplit" << std::endl;
    std::string name = "/" + portName + "/eBottle:i";
    bool r = BufferedPort<emorph::eBottle>::open(name.c_str());

    //r = r && imgWriter.open(std::string(portName + "yarpImage:o").c_str());

    return r;

}

void eReadAndSplit::close()
{
    std::cout << "Closing BufferedPort::eReadAndSplit" << std::endl;
    BufferedPort<emorph::eBottle>::close();
}



void eReadAndSplit::interrupt()
{
   BufferedPort<eBottle>::interrupt();
}


void eReadAndSplit::onRead(emorph::eBottle &incoming)
{
    emorph::eEventQueue q;
    emorph::eEventQueue::iterator qi;

    incoming.getAllSorted(q);
    for(qi = q.begin(); qi != q.end(); qi++) {
        //only display the event if we have define to display events on this
        //channel
        if(eframes->find((*qi)->getChannel())!= eframes->end()) {
            (*eframes)[(*qi)->getChannel()]->addEvent((**qi));
        }
    }
    //eImage->addEvents(incoming);
}


/*////////////////////////////////////////////////////////////////////////////*/
//eFramerModule
/*////////////////////////////////////////////////////////////////////////////*/
eFramerModule::~eFramerModule()
{
    std::map<int, eFrame *>::iterator ii;
    for(ii = eframes.begin(); ii != eframes.end(); ii++)
        delete ii->second;
    std::map<int, yarp::os::BufferedPort<
        yarp::sig::ImageOf<yarp::sig::PixelBgr> > *>::iterator oi;
    for(oi = outports.begin(); oi != outports.end(); oi++)
        delete oi->second;

}

bool eFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //read in config file


    //set the module name for port naming
    moduleName = rf.find("name").asString();
    if(moduleName == "")
        moduleName = "eFramerModule";
    setName(moduleName.c_str());

    //set the eFrame options (sensor size and eventlife
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

    //set the channel list
    yarp::os::Bottle tempList; tempList.addInt(0); tempList.addInt(1);
    yarp::os::Bottle * channelList = rf.find("channels").asList();
    if(!channelList)
        channelList = &tempList;

    //for each channel open an eFrame and an output port
    std::stringstream portNamer;
    for(int i = 0; i < channelList->size(); i++) {
        int c = channelList->get(i).asInt();
        eframes[c] = new eAddressFrame(c, retinaWidth, retinaHeight);
        eframes[c]->setEventLife((int)eventWindow);
        //eframes[i]->setPublishSize(256, 256);
        outports[c] = new yarp::os::BufferedPort<
                yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
        portNamer.str("");
        portNamer << "/" << moduleName << "/ch" << c << ":o";
        outports[c]->open(portNamer.str());
    }

    //open our event reader given the channel list
    eReader = new eReadAndSplit(moduleName);
    eReader->setFrameSet(&eframes);
    eReader->open();

    //set up the frameRate
    period = rf.find("frameRate").asInt();
    if(period == 0) period = 30;
    period = 1.0 / period;

    //set the output image size
    publishHeight = rf.find("publishheight").asInt();
    if(!publishHeight) publishHeight = 256;
    publishWidth = rf.find("publishwidth").asInt();
    if(!publishWidth) publishWidth = 256;

    return true;

}

bool eFramerModule::interruptModule()
{
    eReader->interrupt();
    return true;
}

bool eFramerModule::close()
{
    eReader->close();
    delete eReader;

    return true;
}

bool eFramerModule::respond(const yarp::os::Bottle& command,
                            yarp::os::Bottle& reply)
{
    //add respond messages
    return true;
}

bool eFramerModule::updateModule()
{
    //perhaps do a publish here depending on type of image out
    std::map<int, eFrame *>::iterator mi;
    for(mi = eframes.begin(); mi != eframes.end(); mi++) {
        //get a pointer to buffer space
        yarp::sig::ImageOf<yarp::sig::PixelBgr> &yarpImage =
                outports[(mi->first)]->prepare();

        //set the image size
        yarpImage.resize(publishWidth, publishHeight);

        //make a pointer as a cv::Mat
        cv::Mat publishMat((IplImage *)yarpImage.getIplImage(), false);

        //get the eframer to draw the image
        (mi->second)->publish(publishMat, period);

        //write the image to the port
        outports[mi->first]->write();

    }
    return true;
}

double eFramerModule::getPeriod()
{
    return period;
}

} //namespace emorph
