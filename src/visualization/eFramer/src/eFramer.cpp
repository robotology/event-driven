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

#include "iCub/eFramer.h"
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

    eventLife = 1000; //default 0.001 second
    eTime = 0; //start high to always reset on first event read

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

    //eTime += seconds * 1000000;

    //here we can check for the subset of events to draw.
    cv::Mat canvas = draw(q);
    q.clear(); //this should be remove when proper event persistance is made

    cv::Mat correctChannels(canvas.size(), CV_8UC3);
    if(canvas.type() == CV_8U) {
        cv::cvtColor(canvas, correctChannels, CV_GRAY2BGR);
    } else {
        correctChannels = canvas; //shallow copy
    }

    cv::flip(correctChannels, correctChannels, 0);


    //cannot gaurantee the size or the image type returned so we need to
    //check and convert

    cv::resize(correctChannels, imageOnThePort,
               imageOnThePort.size(), 0, 0, cv::INTER_NEAREST);


}

void eFrame::addEvent(emorph::eEvent &event)
{
    if (event.getStamp() < eTime)
        eTime = event.getStamp();

    eEvent * newcopy = emorph::createEvent(event.getType());
    *newcopy = event;
    q.push_back(newcopy);

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

    //std::cout << "Drawing " << eSet.size() << std::endl;
    emorph::eEventQueue::iterator qi;
    cv::Mat canvas(retinaWidth, retinaHeight, CV_8U);
    canvas.setTo(128);

    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(aep) {
            if(aep->getPolarity())
                canvas.at<char>(aep->getX(), aep->getY()) = 255;
            else
                canvas.at<char>(aep->getX(), aep->getY()) = 0;
            //std::cout << aep->getX() << " " <<aep->getY() << std::endl;
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

    //yarpImage = imgWriter.prepare(); //check this works as it is a return by ref

    //yarpImage.resize(256, 256);
    //eImage = new eAddressFrame(128, 128);
    //eImage->setPublishSize(256, 256);

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

    moduleName = rf.find("name").asString();
    if(moduleName == "")
        moduleName = "eFramerModule";
    setName(moduleName.c_str());

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

    yarp::os::Bottle tempList; tempList.addInt(0); tempList.addInt(1);
    yarp::os::Bottle * channelList = rf.find("channels").asList();
    if(!channelList)
        channelList = &tempList;


    //set up robot etc.

    //set up the frames for each channel
    //yarp::os::Bottle *channel_list = rf.find("channels").asList();

    //if(channel_list->isNull()) {
    std::stringstream portNamer;
    for(int i = 0; i < channelList->size(); i++) {
        int c = channelList->get(i).asInt();
        eframes[c] = new eAddressFrame(c, retinaWidth, retinaHeight);
        //eframes[i]->setPublishSize(256, 256);
        outports[c] = new yarp::os::BufferedPort<
                yarp::sig::ImageOf<yarp::sig::PixelBgr> >;
        portNamer.str("");
        portNamer << "/" << moduleName << "/ch" << c << ":o";
        outports[c]->open(portNamer.str());
    }
    //}

    //set up the reader and splitter
    eReader = new eReadAndSplit(moduleName);
    eReader->setFrameSet(&eframes);
    eReader->open();

    //set up the frameRate
    period = rf.find("frameRate").asInt();
    if(period == 0) period = 10;
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

        //openCV debug window
//        std::stringstream windowname;
//        windowname << "DEBUG C" << mi->first;
//        cv::imshow(windowname.str().c_str(), publishMat);
//        cv::waitKey(1);

//        std::cout << publishMat.rows << " " << publishMat.cols << " " <<
//                     publishMat.channels() << " " << publishMat.depth() <<
//                     std::endl;

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
