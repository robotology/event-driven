/*//////////////////////////////////////////////////////////////////////////////
 *
 * Copyright (C) 2014 iCub Facility, Italian Institute of Technology
 * Author: Arren Glover <arren.glover@iit.it>
 *
 *
 * ///////////////////////////////////////////////////////////////////////////*/

#ifndef __eFramer__
#define __eFramer__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>

namespace emorph {

//forward declaration of the collector. It performs onRead operations to
//add new events to the frame
class eFrame {

protected:

    int publishWidth;
    int publishHeight;
    cv::Mat rawImage;

public:

    eFrame(int retinaWidth, int retinaHeight);

    void setPublishSize(int width, int height);

    virtual void addEvent(emorph::eEvent &event) = 0;
    void clear();

    yarp::sig::ImageOf<yarp::sig::PixelMono> publish();

};

/*!
 * \brief A class that draws a mono image as events are added to it
 */

class eAddressFrame : public eFrame {

public:

    eAddressFrame(int retinaWidth, int retinaHeight) :
        eFrame(retinaWidth, retinaHeight) {}

    //eAddressFrame(int retinaWidth, int retinaHeight) :
        //eFrame(retinaWidth, retinaHeight) {}
    virtual void addEvent(emorph::eEvent &event);

    //~eAddressFrame() {}

};

/*!
 * \brief A class that handles reading events and how long to compile an image.
 * Events are sent to an eImage which controls how events should be drawn.
 *
 */

class eFramerProcess : public yarp::os::BufferedPort<emorph::eBottle>
{
    //has an onRead() function that updates an eImage based on the draw
    //functions and then outputs the image at a certain rate

private:
    std::string portName;

    double period;
    double current_period;

    eFrame * eImage;
    yarp::sig::ImageOf<yarp::sig::PixelMono> yarpImage;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imgWriter;

public:

    eFramerProcess(const std::string &moduleName);
    ~eFramerProcess();

    virtual bool open();
    virtual void close();
    virtual void interrupt();

    void setPeriodMS(int period) { this->period = period; }
    void setWindowSize(int width, int height);

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

    eFramerProcess * eframer;

    //all options for this module
    //retina size (eImage needs to know and it needs to match the hardware)
    //image size (user option but eImage needs it to publish)
    //eventlife (
    //framerate (this module needs to know but it should also be twice the

public:

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

