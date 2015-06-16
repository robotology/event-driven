/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <iCub/ctrl/kalman.h>
#include <fstream>
#include "vCircleObserver.h"

class vCircleReader : public yarp::os::BufferedPort<emorph::vBottle>
{
private:
    
    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    //our circle position estimator

    int inlierThreshold;

    emorph::vtsHelper unwrap;
    double pTS;
    //our filter/tracker
    double periodstart;
    bool debugFlag;

public:

    //we actually allow our observers and trackers
    vCircleObserver circleFinder;
    vCircleTracker circleTracker;
    
    vCircleReader();

    bool    open(const std::string &name);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);

    void setDebug(bool debugFlag) { this->debugFlag = debugFlag; }
    void resetObserverParams(int width, int height, double aDec, double aInj,
                             int aRad, int oWin, int oTrim);
    void resetFilterParams(double pvp, double pvs, double mvp, double mvs);

};

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
//empty line to make gcc happy
