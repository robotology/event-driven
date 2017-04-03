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

#ifndef __VVERGENCE__
#define __VVERGENCE__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/eventdriven/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode2.h>
#include "gaborfilters.h"

class vVergenceManager : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeOut;
    yarp::os::BufferedPort<yarp::os::Bottle> scopeFiltersOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > debugOut;

    //representation for the events
    ev::vSurface2 *fifoLeft;
    ev::vSurface2 *fifoRight;
    ev::vSurface2 *fifoCurr;

    //filters
    std::vector<gaborfilter> filters;
    std::vector<double> filterweights;

    //gaze controller
    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

    //encoders controller
    yarp::dev::PolyDriver encdriver;
    yarp::dev::IEncoders *enccontrol;
    yarp::dev::IPositionControl *poscontrol;
    yarp::dev::IVelocityControl *velcontrol;
    yarp::dev::IControlMode2 *controlmode;
    std::vector<double> encs;
//    double desiredvergence;
    double currvel;

    double depth;

    int width;
    int height;
    int winsize;
    int numberOri;
    int numberPhases;
    double threshold;

    double totweights;
    double error_d;
    double errorPrev;
    double kp;
    double kd;

    //flag to start the vergence
    bool doVergence;

public:

    vVergenceManager(int width, int height, int nEvents, int numberOri, int numberPhases, int maxDisparity, double stdsPerLambda, double threshold);

    void startVerging();
    void resetVergence();

    bool open(const std::string &name, bool strictness);
    void close();
    void interrupt();

    void setkp(double kp) {this->kp = kp;}
    void setkd(double kd) {this->kd = kd;}

    //this is the entry point to your main functionality
    void onRead(ev::vBottle &bot);

};

class vVergenceModule : public yarp::os::RFModule
{
    //rpc port
    yarp::os::RpcServer rpcOut;

    //the event bottle input and output handler
    vVergenceManager      *vergenceManager;



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
