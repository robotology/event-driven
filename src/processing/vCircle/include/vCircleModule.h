/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#ifndef __VCIRCLE__
#define __VCIRCLE__

#include <fstream>
#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <iCub/ctrl/kalman.h>
#include "vCircleObserver.h"


/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEREADER
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleReader : public yarp::os::BufferedPort<emorph::vBottle>
{
private:
    
    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > houghOut;

    emorph::vtsHelper unwrap;
    double pTS;

    //our filter/tracker

    bool strictness;
    bool debugFlag;
    std::ofstream datawriter;
    yarp::os::Stamp pstamp;

public:

    //we actually allow our observers and trackers
    vHoughCircleObserver houghFinder;
    vCircleTracker circleTracker;
    vCircleMultiSize * cObserver;
    double inlierThreshold;
    bool hough;
    double timecounter;
    
    vCircleReader();

    bool    open(const std::string &name, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &inBot);

    bool setDataWriter(std::string datafilename);

};

/*////////////////////////////////////////////////////////////////////////////*/
//VCIRCLEMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class vCircleModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vCircleReader      circleReader;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
