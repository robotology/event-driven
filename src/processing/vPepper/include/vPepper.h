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

/// \defgroup processing Processing
/// \defgroup vPepper vPepper
/// \ingroup processing
/// \brief remove salt and pepper noise

#ifndef __ICUB_DPEPPER_MOD_H__
#define __ICUB_DPEPPER_MOD_H__

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>

class dPepperIO : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    emorph::temporalSurface leftWindow;
    emorph::temporalSurface rightWindow;

    //paramters
    int height;
    int width;
    double spatialSize;
    double temporalSize;

public:

    dPepperIO();

    bool open(const std::string &name);
    void close();
    void interrupt();

    //this is the entry point to your main functionality
    void onRead(emorph::vBottle &bot);

    void setTemporalSize(double microseconds);
    void setSpatialSize(double pixelradius);
    void setResolution(int height, int width);

};

class dPepperModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    dPepperIO      eventManager;


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
