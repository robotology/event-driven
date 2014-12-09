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

#ifndef __eFramer__
#define __eFramer__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>
#include <map>

namespace emorph {

//forward declaration of the collector. It performs onRead operations to
//add new events to the frame
class eFrame {

private:
    emorph::eEventQueue q;

    int channel;
    int retinaWidth;
    int retinaHeight;

    int eventLife;
    yarp::os::Semaphore mutex;

protected:

    virtual cv::Mat draw(emorph::eEventQueue &eSet) = 0;
    int getRetinaWidth() { return retinaWidth; }
    int getRetinaHeight() { return retinaHeight; }

public:

    eFrame(int channel, int retinaWidth, int retinaHeight);

    void setEventLife(int eventLife);

    void addEvent(emorph::eEvent &event);

    void publish(cv::Mat &imageOnThePort, double seconds);

};

/*!
 * \brief A class that draws a mono image as events are added to it
 */

class eAddressFrame : public eFrame {

public:

    eAddressFrame(int channel, int retinaWidth, int retinaHeight) :
        eFrame(channel, retinaWidth, retinaHeight) {}

    //eAddressFrame(int retinaWidth, int retinaHeight) :
        //eFrame(retinaWidth, retinaHeight) {}
    virtual cv::Mat draw(eEventQueue &eSet);

    //~eAddressFrame() {}

};

/*!
 * \brief A class that handles reading events and how long to compile an image.
 * Events are sent to an eImage which controls how events should be drawn.
 *
 */

class eReadAndSplit : public yarp::os::BufferedPort<emorph::eBottle>
{
    //has an onRead() function that updates an eImage based on the draw
    //functions and then outputs the image at a certain rate

private:
    std::string portName;
    std::map<int, eFrame *> *eframes;

public:

    eReadAndSplit(const std::string &moduleName);

    void setFrameSet(std::map<int, eFrame *> *eframes)
        {this->eframes = eframes;}

    virtual bool open();
    virtual void close();
    virtual void interrupt();
    virtual void onRead(emorph::eBottle &incoming);


};

/*!
 * \brief The module which envolopes making frames from event-based data
 */

class eFramerModule : public yarp::os::RFModule {

private:

    std::string moduleName;         // name of the module (rootname of ports)
    std::string robotName;          // name of the robot
    std::string robotPortName;      // reference to the head of the robot
    std::string rpcPortName;    // name for comunication with respond

    double period;
    int publishWidth;
    int publishHeight;

    //this is the eBottle reading port
    eReadAndSplit * eReader;

    //this is the image frame drawers
    std::map<int, eFrame *> eframes;

    //this is the output ports sending the yarp::Imageofs on
    std::map<int, yarp::os::BufferedPort<
        yarp::sig::ImageOf<yarp::sig::PixelBgr> > *> outports;

    //all options for this module
    //retina size (eImage needs to know and it needs to match the hardware)
    //image size (user option but eImage needs it to publish)
    //eventlife (
    //framerate (this module needs to know but it should also be twice the

public:

    virtual ~eFramerModule();

    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the module
    virtual bool respond(const yarp::os::Bottle& command,
                         yarp::os::Bottle& reply);

    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
};


} //namespace emorph

#endif //eframer

