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
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IVelocityControl.h>
#include <deque>
#include <vector>

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
  VELOCITY CONTROL (WITHOUT GAZECONTROLLER)
  ////////////////////////////////////////////////////////////////////////////*/
//PID by Ugo Pattacini
class PID
{
    double Kp,Ki;
    double integral;

public:
    // constructor
    PID() : Kp(0.0), Ki(0.0), integral(0.0) { }

    // helper function to set up sample time and gains
    void set(const double Kp, const double Ki)
    {
        this->Kp=Kp;
        this->Ki=Ki;
    }

    // compute the control command
    double command(const double reference, const double feedback, const double Ts)
    {
        // the actual error between reference and feedback
        double error=reference-feedback;

        // accumulate the error
        integral+=error*Ts;

        // compute the PID output
        return (Kp*error+Ki*integral);
    }

    void reset()
    {
        integral = 0;
    }
};

class eyeControlPID
{

protected:

    yarp::dev::PolyDriver         driver;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IVelocityControl *ivel;

    int nAxes;
    std::vector<PID*> controllers;
    std::vector<double> velocity;
    std::vector<double> encs;

    int u_fixation;
    int v_fixation;

public:

    eyeControlPID() : nAxes(6), u_fixation(0), v_fixation(0) {}

    bool initialise(int height, int width)
    {
        u_fixation = width / 2;
        v_fixation = height / 2;

        yarp::os::Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icub/head");
        option.put("local","/controller");

        if (!driver.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        // open the views

        if(!driver.view(ienc)) {
            yError() << "Driver does not implement encoder mode";
            return false;
        }
        if(!driver.view(ivel)) {
            yError() << "Driver does not implement velocity mode";
            return false;
        }

        // retrieve number of axes
        int readAxes;
        ienc->getAxes(&readAxes);
        if(readAxes != nAxes) {
            yError() << "Incorrect number of axes" << readAxes << nAxes;
            return false;
        }

        velocity.resize(nAxes);
        encs.resize(nAxes);
        controllers.resize(nAxes);
        for(int i = 0; i < nAxes; i++)
            controllers[i] = new PID;

        // set up our controllers
        controllers[0]->set(3.0, 0.1);
        controllers[1]->set(0.0, 0.0);
        controllers[2]->set(3.0, 0.1);
        controllers[3]->set(2.0, 0.1);
        controllers[4]->set(2.0, 0.1);
        controllers[5]->set(0.2, 0.0);

        //set velocity control mode
        yarp::dev::IControlMode *imod;
        if(!driver.view(imod)) {
            yError() << "Driver does not implement control mode";
            return false;
        }

        std::vector<int> modes(nAxes, VOCAB_CM_VELOCITY);
        if(!imod->setControlModes(modes.data())) {
            yError() << "Could not set velocity control mode";
            return false;
        }

        return true;

    }

    void controlMono(int u, int v, double dt)
    {
        ienc->getEncoders(encs.data());

        double eyes_pan=-controllers[4]->command(u_fixation, u, dt);
        double eyes_tilt=controllers[3]->command(v_fixation, v, dt);
        controllers[2]->reset();
        double eyes_ver=-controllers[5]->command(0, 0, dt);

        double neck_tilt=-controllers[0]->command(0.0,encs[3], dt);
        double neck_pan=controllers[2]->command(0.0,encs[4], dt);

        // send commands to the robot head
        velocity[0]=neck_tilt;          // neck pitch
        velocity[1]=0.0;                // neck roll
        velocity[2]=neck_pan;           // neck yaw
        velocity[3]=eyes_tilt;          // eyes tilt
        velocity[4]=eyes_pan;           // eyes pan
        velocity[5]=eyes_ver;           // eyes vergence
        ivel->velocityMove(velocity.data());
    }

    void controlStereo(int ul, int vl, int ur, int vr, double dt)
    {
        ienc->getEncoders(encs.data());

        double eyes_pan=-controllers[4]->command(u_fixation, ul, dt);
        double eyes_tilt=controllers[3]->command(v_fixation, vl, dt);
        double eyes_ver=-controllers[5]->command(0,ul-ur, dt);

        // feed-forward correction to reduce
        // interplay between vergence and pan
        eyes_pan-=eyes_ver/2.0;

        double neck_tilt=-controllers[0]->command(0.0,encs[3], dt);
        double neck_pan=controllers[2]->command(0.0,encs[4], dt);

        // send commands to the robot head
        velocity[0]=neck_tilt;          // neck pitch
        velocity[1]=0.0;                // neck roll
        velocity[2]=neck_pan;           // neck yaw
        velocity[3]=eyes_tilt;          // eyes tilt
        velocity[4]=eyes_pan;           // eyes pan
        velocity[5]=eyes_ver;           // eyes vergence
        ivel->velocityMove(velocity.data());
    }

    void controlReset()
    {
        for(int i = 0; i < nAxes; i++) {
            controllers[i]->reset();
            velocity[i] = 0;
        }
        ivel->velocityMove(velocity.data());
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
    yarp::os::BufferedPort<yarp::sig::Vector> debugOutPort;
    yarp::sig::Vector arm_target_position;

    //the remote procedure port
    yarp::os::RpcServer rpcPort;

    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    eyeControlPID velocityController;

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
    bool velocityControl;

    double period;

    bool controlCartesian(yarp::sig::Vector ltarget, yarp::sig::Vector rtarget);
    bool controlArm(yarp::sig::Vector ltarget, yarp::sig::Vector rtarget);
    bool controlVelocity(yarp::sig::Vector ltarget, yarp::sig::Vector rtarget);
    bool controlExternal(yarp::sig::Vector ltarget, yarp::sig::Vector rtarget);

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
