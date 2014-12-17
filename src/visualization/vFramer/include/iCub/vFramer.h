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

#ifndef __vFramer__
#define __vFramer__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>
#include <map>

namespace emorph {

/*!
 * \brief The vFrame class
 */
class vFrame {

private:
    emorph::vQueue q;

    int channel;
    int retinaWidth;
    int retinaHeight;

    int eventLife;
    yarp::os::Semaphore mutex;

protected:

    virtual cv::Mat draw(emorph::vQueue &eSet) = 0;
    int getRetinaWidth() { return retinaWidth; }
    int getRetinaHeight() { return retinaHeight; }

public:

    vFrame(int channel, int retinaWidth, int retinaHeight);

    void setEventLife(int eventLife);

    void addEvent(emorph::vEvent &event);

    void publish(cv::Mat &imageOnThePort);

};

class vWindow {

private:
    emorph::vQueue q;
    int windowSize;
    yarp::os::Semaphore mutex;

public:
    vWindow(int windowSize = 50000) { this->windowSize = windowSize; }
    void addEvent(emorph::vEvent &event);
    int getCurrentWindow(emorph::vQueue q);

};

/*!
 * \brief The eAddressFrame class
 */
class eAddressFrame : public vFrame {

public:

    eAddressFrame(int channel, int retinaWidth, int retinaHeight) :
        vFrame(channel, retinaWidth, retinaHeight) {}

    //eAddressFrame(int retinaWidth, int retinaHeight) :
        //vFrame(retinaWidth, retinaHeight) {}
    virtual cv::Mat draw(vQueue &eSet);

    //~eAddressFrame() {}

};



/*!
 * \brief The vReadAndSplit class
 */

class vReadAndSplit : public yarp::os::BufferedPort<emorph::vBottle>
{
    //has an onRead() function that updates an eImage based on the draw
    //functions and then outputs the image at a certain rate

private:
    std::string portName;
    std::map<int, vFrame *> *vFrames;

public:

    vReadAndSplit(const std::string &moduleName);

    void setFrameSet(std::map<int, vFrame *> *vFrames)
        {this->vFrames = vFrames;}

    virtual bool open();
    virtual void close();
    virtual void interrupt();
    virtual void onRead(emorph::vBottle &incoming);


};

/*!
 * \brief The module which envolopes making frames from event-based data
 */

class vFramerModule : public yarp::os::RFModule {

private:

    std::string moduleName;         // name of the module (rootname of ports)
    std::string robotName;          // name of the robot
    std::string robotPortName;      // reference to the head of the robot
    std::string rpcPortName;    // name for comunication with respond

    double period;
    int publishWidth;
    int publishHeight;

    //this is the vBottle reading port
    vReadAndSplit * vReader;

    //this is the image frame drawers
    std::map<int, vFrame *> vFrames;

    //this is the output ports sending the yarp::Imageofs on
    std::map<int, yarp::os::BufferedPort<
        yarp::sig::ImageOf<yarp::sig::PixelBgr> > *> outports;

    //all options for this module
    //retina size (eImage needs to know and it needs to match the hardware)
    //image size (user option but eImage needs it to publish)
    //eventlife (
    //framerate (this module needs to know but it should also be twice the

public:

    virtual ~vFramerModule();

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

#endif //vFramer

