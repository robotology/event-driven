/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren.Glover@iit.it
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

/// \defgroup robotBehaviour robotBehaviour
/// \defgroup vTrackToRobot vTrackToRobot
/// \ingroup robotBehaviour
/// \brief perform gaze control given cluster track events

#ifndef __ICUB_VTRACKTOROBOT_H__
#define __ICUB_VTRACKTOROBOT_H__

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <deque>

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

class vTrackToRobotManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    yarp::os::BufferedPort<yarp::os::Bottle> cartOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> positionOutPort;
    yarp::os::BufferedPort<emorph::vBottle> eventsOutPort;
    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    enum { fromgaze, fromsize, fromstereo };
    int method;
    bool gazingActive;
    enum { gazedemo, graspdemo };
    int demo;
    double lastdogazetime;


    emorph::temporalSurface FIFO;
    std::deque<yarp::sig::Vector> recentgazelocs;
    std::deque<double> recenteyezs;
    double p_eyez;
    double medx;
    double medy;
    yarp::sig::Vector xrobref; //this stores the gaze position in eye ref frame
    yarp::sig::Vector px; //the pixel position to make a gaze

public:

    vTrackToRobotManager();

    void setMethod(std::string methodname);
    void setDemo(std::string demoname);
    void startGazing() {gazingActive = true;}
    void stopGazing() {gazingActive = false;}

    bool open(const std::string &name);
    void onRead(emorph::vBottle &bot);
    void interrupt();
    void close();

};

/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

class vTrackToRobotModule : public yarp::os::RFModule
{
private:

    //the event bottle input and output handler
    vTrackToRobotManager      vTrackToRobot;

    //the remote procedure port
    yarp::os::RpcServer     rpcPort;


    //robot control settings
//    yarp::dev::PolyDriver mdriver;
//    yarp::dev::IPositionControl *pc;
//    yarp::dev::IEncoders *ec;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);

};


#endif
//empty line to make gcc happy
