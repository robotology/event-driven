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
#include <yarp/dev/all.h>

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

class vTrackToRobotManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:
    
    yarp::os::BufferedPort<yarp::os::Bottle> cartOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> positionOutPort;
    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    enum { fromgaze, fromsize, fromstereo };

    int method;

public:
    
    vTrackToRobotManager();

    bool setMethod(std::string methodname);

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

};


#endif
//empty line to make gcc happy
