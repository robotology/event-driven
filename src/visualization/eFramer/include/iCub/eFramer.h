/*//////////////////////////////////////////////////////////////////////////////
 *
 * Copyright (C) 2014 iCub Facility, Italian Institute of Technology
 * Author: Arren Glover <arren.glover@iit.it>
 *
 *
 * ///////////////////////////////////////////////////////////////////////////*/


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/eBottle.h>
#include <iCub/emorph/eCodec.h>

//forward declaration of the collector. It performs onRead operations to
//add new events to the frame
class eFramerCollector;

class eFramerModule : public yarp::os::RFModule {

private:

    std::string moduleName;         // name of the module (rootname of ports)
    std::string robotName;          // name of the robot
    std::string robotPortName;      // reference to the head of the robot
    std::string handlerPortName;    // name for comunication with respond

    int framerate;                  // rate at which frames are produced
    int retinaWidth;                // number of pixels
    int retinaHeight;               // number of pixels
    int windowSize;                 // display window size

    yarp::sig::ImageOf<yarp::sig::PixelMono> frame;

    //input port for eBottles
    yarp::os::BufferedPort<emorph::eBottle> portIn_eBottle;

    //output port for images
    yarp::os::BufferedPort< yarp::sig::ImageOf
                     < yarp::sig::PixelMono > > portOut_image;

    //input port for messages
    yarp::os::Port handlerPort;     // a port to handle messages

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
};


class eFramerCollector : public yarp::os::BufferedPort<emorph::eBottle>
{



};
