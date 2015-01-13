//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse                                                                                                                                                                                //
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//

#ifndef AQUILA_LITE_ICUBMOTOR_H
#define AQUILA_LITE_ICUBMOTOR_H

#define MAX_SIM_HEAD_VELOCITY  50
#define MAX_REAL_HEAD_VELOCITY 40
#define MAX_SIM_LEG_VELOCITY  10
#define MAX_REAL_LEG_VELOCITY 10
#define MAX_TORSO_VELOCITY 10


#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>
//#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

#include <string>
#include <stdio.h>
#include <iostream>


#define CTRL_THREAD_PER     0.01    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]
#define T					2.4
#define XD					-0.1


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

namespace aquilacubmotor
{

YARP_DECLARE_DEVICES(icubmod)

class CubCartThread: public RateThread,
                     public CartesianEvent
{
protected:

    bool simulationMode;
    PolyDriver         Lclient;
    PolyDriver         Rclient;
    ICartesianControl *icartL;
    ICartesianControl *icartR;
    Port inputPort;

    Vector Lxd, Rxd, x0;
    Vector Lod, Rod, o0;

    int startup_context_left_id, startup_context_right_id;

    double t;
    double t0;
    double t1;

    virtual void cartesianEventCallback();

public:

    CubCartThread(const double period, bool simMode);
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    void getTarget();
    void generateTarget();
    double norm(const Vector &v);
    void limitTorsoPitch();
    void printStatus();

};

class CubCart: public RFModule
{
protected:
    CubCartThread *thr;
    bool simulationMode;
public:
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();

    virtual double getPeriod();
    virtual bool   updateModule();
    void setSimulationMode(bool simMode);
};

class ICubMotor : public yarp::os::Thread
{

public:
    ICubMotor();
    ICubMotor(bool simMode, bool cartMode);
    ~ICubMotor();

protected:

    //CubCart cartMod;
    CubCartThread *thr;
    ResourceFinder rf;

private:
    bool simulationMode;
    bool cartesianMode;
    bool running;

    void run();

    BufferedPort<Bottle> pointPort;
    std::string pointPortName;

public:
    //structures
    struct SimplePart
    {
        IControlLimits                  *lim;
        IPositionControl                *pos;
        IVelocityControl                *vel;
        IControlMode                    *mode;
        IEncoders                       *encs;
        IImpedanceControl               *iimp;
        ITorqueControl                  *itrq;
        yarp::sig::Vector               *encoders;
        yarp::sig::Vector               *initpos;    
        yarp::sig::Vector               *command;
        yarp::sig::Vector               *velocity;
        yarp::sig::Vector               *torques;
        yarp::sig::Vector               xd, x0;         //cartesian controller stuff
        yarp::sig::Vector               od, o0;
        yarp::sig::Vector               position;
        PolyDriver                      *device;
        ICartesianControl               *icart;
        int                             startup_context_id;
        int                             num_joints;
        int                             num_clients;
        int                             max_velocity;
        double                          *velocities;
        double                          *positions;
        double                          *min_limit;
        double                          *max_limit;
        bool                            initialised;
    };
    SimplePart *head,*torso,*leftArm,*rightArm,*leftLeg,*rightLeg,*leftCart;

    float prevTorso[3];
    float prevHead[5];
    bool terminalMode;
    bool readyToMove;
    bool moving;
    bool armHome;
    bool cartMove;
    bool tracking;
    bool pickUp;
    bool trackingMotion;
    bool standing;

    void modICubMotor();
    void point();
    void moveTorso();
    void moveArms();
    void reach();
    void reachPickUp();
    void shutLeftHand(double distance, double headPos);
    void shutRightHand(double distance, double headPos);
    void pointLeftHand();
    void pointRightHand();
    void moveHead(float target_x, float target_y);
    void moveEyes(float target_x, float target_y);
    void resetHead();
    void sitDown();
    void lookDown();
    void lookLeft();
    void lookRight();
    void trackMotion();
    void headTracking();
    void initHead(bool simulation);    
    void initleftLeg(bool simulation);
    void initrightLeg(bool simulation);
    void initTorso(bool simulation);
    void initCartesian(bool simulation);
    void closeHead();
    void closeLeftLeg();
    void closeRightLeg();
    void closeTorso();
    void closeImpedance();
    void closeCartesian();
    int initImpedance(bool simulation);
    int  getDifference(int a, int b);


    void clean();
    void connectPorts();
    void disconnectPorts();
    bool openPorts(std::string portPrefix);
    void stop();
    void setSimulationMode(bool simON);
    bool getSimulationMode();
    void setCartesianMode(bool cartON);
    bool getCartesianMode();

};

}
#endif//AQUILA_LITE_ICUBMOTOR_H
