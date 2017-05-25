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

/// \defgroup Modules Modules
/// \defgroup vPepper vPepper
/// \ingroup Modules
/// \brief removes salt-and-pepper noise from the event stream

#ifndef __VPEPPERMODULE__
#define __VPEPPERMODULE__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
using namespace::ev;

class vPepperIO : public yarp::os::Thread
{
private:

    //output port for the vBottle with the new events computed by the module
    ev::queueAllocator inPort;
    yarp::os::BufferedPort<ev::vBottle> outPort;

    //filter class
    ev::vNoiseFilter thefilter;

    //parameters
    std::string name;

public:

    vPepperIO() : name("/vPepper") {}
    ~vPepperIO();
    void initialise(std::string name, int height, int width, int spatialSize,
                    int temporalSize);
    void run();
    void onStop();
    bool threadInit();

};

class vPepperModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vPepperIO      eventManager;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();

    virtual double getPeriod();
    virtual bool updateModule();

};


#endif

