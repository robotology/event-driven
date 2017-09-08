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
#include <yarp/dev/CartesianControl.h>
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
        leftTarget.resize(4);
        rightTarget.resize(4);
        leftTarget[2] = -100;
        rightTarget[2] = 100;
        leftTarget[3] = 0;
        rightTarget[3] = 0;
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
                if(v->polarity == 0) {
                    rightTarget[3] = 0;
                } else {
                    rightTarget[0] = v->x;
                    rightTarget[1] = v->y;
                    rightTarget[2] = v->sigx; //radius
                    rightTarget[3] = 1;
                }
            } else if(v->channel == VLEFT && !leftupdated) {
                leftupdated = true;
                if(v->polarity == 0) {
                    leftTarget[3] = 0;
                } else {
                    leftTarget[0] = v->x;
                    leftTarget[1] = v->y;
                    leftTarget[2] = v->sigx; //radius
                    leftTarget[3] = 1;
                }
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

    ev::resolution res;
    //the event bottle input and output handler
    positionReader inputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> cartOutPort;

    //the remote procedure port
    yarp::os::RpcServer rpcPort;

    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    yarp::sig::Vector armhomepos, armhomerot;
    yarp::sig::Vector headhomepos, headhomerot;
    bool usearm;
    yarp::dev::PolyDriver armdriver;
    yarp::dev::ICartesianControl *arm;
    int startup_context_id;

    double yThresh;
    double rThresh;
    bool gazingActive;
    bool useDemoRedBall;

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
