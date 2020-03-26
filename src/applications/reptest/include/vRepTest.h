/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// \defgroup Modules Modules
// \defgroup vRepTest vRepTest
// \ingroup Modules
// \brief tests different event representation types

#ifndef __REPTEST__
#define __REPTEST__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <event-driven/all.h>
#include <event-driven/deprecated.h>

class vRepTest : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<yarp::os::Bottle> dumper;
    yarp::os::BufferedPort<ev::vBottle> eventsOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf <yarp::sig::PixelBgr> > imPort;

    //for helping with timestamp wrap around
    ev::vtsHelper unwrapper;
    double ytime;

    ev::vEdge edge;
    ev::temporalSurface tWindow;
    ev::fixedSurface fWindow;
    ev::lifetimeSurface lWindow;
    ev::vFuzzyEdge fedge;

    std::string vistype;
    void drawDebug(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image,
                   const ev::vQueue &q, int xoff, int yoff);

public:

    vRepTest();
    void setTemporalWindow(int dt) {tWindow.setTemporalSize(dt);}
    void setFixedWindow(int N) {fWindow.setFixedWindowSize(N);}
    void setVisType(std::string vis) {this->vistype = vis;}

    bool    open(const std::string &name, bool strict = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &inBottle);

};

class vRepTestHandler : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vRepTest      reptest;

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
