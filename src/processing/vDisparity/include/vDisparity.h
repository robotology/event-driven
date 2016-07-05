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

#ifndef __VDISPARITY__
#define __VDISPARITY__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <yarp/dev/PolyDriver.h>
//#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode2.h>
#include "gaborfilters.h"

class vDisparityManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > debugOut;

    //representation for the events
    emorph::vSurface2 *fifoLeft;
    emorph::vSurface2 *fifoRight;
    emorph::vSurface2 *fifoCurr;

    //filters
    std::vector<gaborfilter> filters;
    std::vector<double> filterweights;

//    //gaze controller
//    yarp::dev::PolyDriver gazedriver;
//    yarp::dev::IGazeControl *gazecontrol;

    //encoders controller
    yarp::dev::PolyDriver encdriver;
    yarp::dev::IEncoders *enccontrol;
    yarp::dev::IPositionControl *poscontrol;
    yarp::dev::IVelocityControl *velcontrol;
    yarp::dev::IControlMode2 *controlmode;
    std::vector<double> encs;
    std::vector<double> vels;
    double desiredvergence;


    int width;
    int height;
    int winsize;

public:

    vDisparityManager(int width, int height, int nEvents, int numberOri, int numberPhases, int maxDisparity, double stdsPerLambda);

    bool    open(const std::string &name, bool strictness);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);

};

class vDisparityModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vDisparityManager      *disparityManager;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
