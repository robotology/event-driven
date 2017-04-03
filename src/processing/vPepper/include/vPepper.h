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

#ifndef __ICUB_DPEPPER_MOD_H__
#define __ICUB_DPEPPER_MOD_H__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
using namespace::ev;

class vPepperIO : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;

    ev::vNoiseFilter thefilter;


    //paramters
    ev::resolution res;
    double spatialSize;
    double temporalSize;
    bool strict;

public:

    vPepperIO();

    void initialise(int height, int width, int spatialSize, int temporalSize);

    bool open(const std::string &name, bool strict);
    void close();
    void interrupt();

    //this is the entry point to your main functionality
    void onRead(ev::vBottle &bot);

};

class vPepperModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vPepperIO      eventManager;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();

    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
