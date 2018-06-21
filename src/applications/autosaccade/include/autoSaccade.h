/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           massimiliano.iacono@iit.it
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
// \defgroup autosaccade autosaccade
// \ingroup RobotIO
// \brief automatically elicit iCub eye movement when event rate is low

#ifndef __V_AUTOSACCADE__
#define __V_AUTOSACCADE__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>


class EventBottleManager : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    //for helping with timestamp wrap around
    ev::vtsHelper unwrapper;

    //rate counters
    yarp::os::Semaphore mutex;
    unsigned long int latestStamp;
    unsigned int vCount;
    double yRate;
    bool isReading;
    ev::vQueue vQueue;

public:

    EventBottleManager();

    bool    open(const std::string &name);

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);

    //the getting functions of the parent class
    unsigned long int getTime();
    unsigned long int popCount();
    double getEventRate() { return yRate; }

    ev::vQueue getEvents() ;
    bool start();
    bool stop();

};

class AutoSaccadeModule : public yarp::os::RFModule
{
private:
    //the remote procedure port
    yarp::os::RpcServer     rpcPort;

    //the event bottle input and output handler
    EventBottleManager      eventBottleManager;

    //timing parameters
    double checkPeriod;

    //Minimum event rate to trigger saccade
    double minVpS;

    //robot control settings
    yarp::dev::PolyDriver mdriver;
    yarp::dev::PolyDriver gazeDriver;
    yarp::dev::IGazeControl *gazeControl;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IControlMode     *imod;
    int context0;
    std::string robotName;

    //Camera parameters
    int camWidth, camHeight;


    //saccading parameters
    double refSpeed;
    double refAcc;

    //duration of event collection
    double timeout;

    yarp::os::Port vRatePort;
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr>> leftImgPort;
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr>> rightImagePort;

    //timestamp for comparison
    double prevStamp;

    void performSaccade();
    bool configDriver( int joint, double refSp, double refAcc );
    double computeEventRate();
    bool computeCenterMass( yarp::sig::Vector &cmR, yarp::sig::Vector &cmL, ev::vQueue &q );
    void home();
    bool openJointControlDriver();
    bool openGazeDriver();
    bool openPorts();
    void readParams( const yarp::os::ResourceFinder &rf );
   /* void visualizeEvents( yarp::sig::ImageOf<yarp::sig::PixelBgr> &leftImage,
            yarp::sig::ImageOf<yarp::sig::PixelBgr> &rightImage,
            ev::vQueue &q ) const;*/
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
