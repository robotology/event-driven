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

// \defgroup RobotIO RobotIO
// \defgroup vTrackToRobot vTrackToRobot
// \ingroup RobotIO
// \brief perform gaze control given cluster track events

#ifndef __ICUB_VTRACKTOROBOT_H__
#define __ICUB_VTRACKTOROBOT_H__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <deque>

using namespace ev;

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

class positionReader : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    yarp::sig::Vector leftTarget;
    yarp::sig::Vector rightTarget;

public:

    positionReader()
    {
        leftTarget.resize(3);
        rightTarget.resize(3);
        leftTarget[2] = -100;
        useCallback();
    }

    void onRead(vBottle &vBottleIn)
    {

        //get the Q
        vQueue q = vBottleIn.get<GaussianAE>();
        if(q.empty()) {
            yWarning() << "q empty in callback function?";
            return;
        }

        //update our current best position of the ball in both cameras
        bool leftupdated = false, rightupdated = false;
        vQueue::reverse_iterator qi = q.rbegin();
        while((!leftupdated || !rightupdated) && qi != q.rend()) {
            auto v = is_event<GaussianAE>(*qi);
            if(v->channel == VRIGHT && !rightupdated) {
                rightupdated = true;
                rightTarget[0] = v->x;
                rightTarget[1] = v->y;
                rightTarget[2] = v->sigx; //radius
            } else if(v->channel == VLEFT && !leftupdated) {
                leftupdated = true;
                leftTarget[0] = v->x;
                leftTarget[1] = v->y;
                leftTarget[2] = v->sigx; //radius
            }
            qi++;
        }

    }

    void getTargets(yarp::sig::Vector &left, yarp::sig::Vector &right)
    {
        left = leftTarget;
        right = rightTarget;
    }


};

/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

class vTrackToRobotModule : public yarp::os::RFModule
{
private:

    //the event bottle input and output handler
    positionReader inputPort;

    //the remote procedure port
    yarp::os::RpcServer rpcPort;

    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    double yThresh;
    double rThresh;
    bool gazingActive;

    double period;


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
