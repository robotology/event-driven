// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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

/**
 * @file graspThread.h
 * @brief Definition of a thread that allows the robot to perform actions on the environment
 * (see objectInteractorModule.h).
 */

#ifndef _GRASP_THREAD_H_
#define _GRASP_THREAD_H_

#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/action/actionPrimitives.h>
#include <iCub/ActionPrimitivesLayer3.h>
#include <yarp/sig/all.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>
#include <map>

#define USE_LEFT                    0
#define USE_RIGHT                   1
                                    
#define CMD_TOUCH                   VOCAB4('t','o','u','c')
#define CMD_GRASP                   VOCAB4('g','r','a','s')
#define CMD_TAP                     VOCAB3('t','a','p')
#define CMD_PUSH                    VOCAB4('p','u','s','h')
#define CMD_CALIB                   VOCAB4('c','a','l','i')
                                    
#define OPT_TABLE                   VOCAB4('t','a','b','l')
#define OPT_KIN                     VOCAB3('k','i','n')
#define OPT_START                   VOCAB4('s','t','a','r')
#define OPT_STOP                    VOCAB4('s','t','o','p')
#define OPT_CLEAR                   VOCAB4('c','l','e','a')
#define OPT_LEFT                    VOCAB4('l','e','f','t')
#define OPT_RIGHT                   VOCAB4('r','i','g','h')
#define OPT_ON                      VOCAB2('o','n')
#define OPT_OFF                     VOCAB3('o','f','f')

#define KINCALIB_PERIOD             20      // [ms]
#define HOMING_PERIOD               2.0     // [s]

#ifdef WIN32
    #pragma warning(disable:4996)
#endif

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;


// this class manages the kinematic offsets
class kinematicOffset
{
protected:
    struct offsElement
    {
        Vector point;
        Vector offset;
    };

    deque<offsElement> offsList;
    string fileName;
    bool toBeSaved;

public:
    kinematicOffset()
    {
        fileName="";
        toBeSaved=false;
    }

    void clear()
    {
        if (offsList.size())
        {
            offsList.clear();
            toBeSaved=true;
        }
    }

    void insert(const Vector &point, const Vector &offset)
    {
        offsElement el;
        el.point=point;
        el.offset=offset;

        offsList.push_back(el);
        toBeSaved=true;

        cout<<"########## Kinematic offset added: ["<<el.offset.toString()<<"]"<<endl;
    }

    Vector get(const Vector &in)
    {
        Vector out(3);
        out=0.0;

        if (offsList.size())
        {
            double sumWeight=0.0;

            for (unsigned int i=0; i<offsList.size(); i++)
            {
                Vector &point=offsList[i].point;
                Vector &offset=offsList[i].offset;

                double weight=norm(in-point);

                if (weight)
                    weight=1.0/weight;
                else
                    return out=offset;

                out=out+weight*offset;
                sumWeight+=weight;
            }

            out=(1.0/sumWeight)*out;
        }

        return out;
    }

    
    void load(const string &_fileName)
    {        
        fileName=_fileName;

        Property config;
        config.fromConfigFile(fileName.c_str());
        if (config.isNull())
            return;

        // parsing general part
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
            return;
       
        int numItems=bGeneral.check("numItems",Value(0)).asInt();

        // parsing general config options
        for (int i=0; i<numItems; i++)
        {
            char item[255];
            sprintf(item,"item_%d",i);
            Bottle &bItem=config.findGroup(item);

            if (bItem.isNull())
                continue;

            Vector point, offset;
            point.resize(3,0.0);
            offset.resize(3,0.0);

            if (Bottle *pB=bItem.find("point").asList())
            {
                int sz=pB->size();
                int len=sz<3?sz:3;

                for (int i=0; i<pB->size(); i++)
                    point[i]=pB->get(i).asDouble();
            }

            if (Bottle *pB=bItem.find("offset").asList())
            {
                int sz=pB->size();
                int len=sz<3?sz:3;

                for (int i=0; i<pB->size(); i++)
                    offset[i]=pB->get(i).asDouble();
            }

            insert(point,offset);
        }

        toBeSaved=false;
    }
    

    /*
    void save()
    {
        if (toBeSaved)
        {
            ofstream fout;
            fout.open(fileName.c_str());
    
            // general part
            fout<<endl;
            fout<<"[general]"<<endl;
            fout<<"numItems\t"<<offsList.size()<<endl;
            fout<<endl;
    
            // items part
            for (unsigned int i=0; i<offsList.size(); i++)
            {
                Vector &point=offsList[i].point;
                Vector &offset=offsList[i].offset;
    
                fout<<"[item_"<<i<<"]"<<endl;
                fout<<"point\t("<<point.toString()<<")"<<endl;
                fout<<"offset\t("<<offset.toString()<<")"<<endl;
                fout<<endl;
            }
    
            fout.close();
        }
    }

    */
    ~kinematicOffset()
    {
        //save();
        offsList.clear();
    }
};


// this class handles the offset to be added to the
// current table heigth according to the feedback
// coming from contact detection
class tableManager
{
protected:
    string fileName;
    double heightOffset;

    string id;
    Vector position;

    string sender;
    string receiver;

public:
    tableManager()
    {
        fileName="";
        heightOffset=0.0;

        id="";
        position.resize(3,0.0);

        sender="";
        receiver="";
    }

    void initTxParams(const string &_sender, const string &_receiver)
    {
        sender=_sender;
        receiver=_receiver;
    }

    void load(const string &_fileName)
    {        
        fileName=_fileName;

        Property config;
        config.fromConfigFile(fileName.c_str());
        if (config.isNull())
            return;

        // parsing id option
        id=config.check("id",Value("")).asString().c_str();

        // parsing POSITION part
        Bottle &bPosition=config.findGroup("POSITION");
        if (bPosition.isNull())
            return;

        position[0]=bPosition.check("x",Value(0.0)).asDouble();
        position[1]=bPosition.check("y",Value(0.0)).asDouble();
        position[2]=bPosition.check("z",Value(0.0)).asDouble();

        heightOffset=0.0;
    }

    /*
    void save()
    {
        if (heightOffset)
        {
            ofstream fout;
            fout.open(fileName.c_str());

            // id option
            fout<<endl;
            fout<<"id\t"<<id<<endl;
            fout<<endl;

            // position part
            fout<<"[POSITION]"<<endl;
            fout<<"x\t"<<position[0]<<endl;
            fout<<"y\t"<<position[1]<<endl;
            fout<<"z\t"<<(position[2]+heightOffset)<<endl;

            fout.close();
        }
    }
    */

    void setTableHeight(const double height)
    {
        heightOffset=height-position[2];

        if (heightOffset)
        {
            Port port;
            port.open(sender.c_str());

            if (Network::connect(sender.c_str(),receiver.c_str()))
            {
                Bottle cmd, reply;
                cmd.addString("set");
                cmd.addString("heightOffset");
                cmd.addDouble(heightOffset);
                   
                port.write(cmd,reply);
            }
            else
                cout<<"Warning: unable to connect to "<<receiver<<endl;

            port.close();
        }
    }

    ~tableManager()
    {
        //save();
    }
};


// this class handles the manual calibration of the arm
// with the goal to find out the kinematic offsets
class kinematicCalibrator : public RateThread
{
protected:
    ActionPrimitivesLayer2 *action;

    double forceThres;
    double gain;

    Vector x0;
    Vector o0;
    Vector x;

public:
    kinematicCalibrator() : RateThread(KINCALIB_PERIOD)
    {
        action=NULL;
        forceThres=1e9;
        gain=0.0;

        x0.resize(3,0.0);
        o0.resize(4,0.0);
        x.resize(3,0.0);
    }

    void init(ActionPrimitivesLayer2 *_action, const Vector &_x0,
              const double thres)
    {
        action=_action;
        forceThres=thres;

        x0=_x0;
        action->getPose(x,o0);
    }

    void   set_gain(const double _gain) { gain=_gain;    }
    Vector get_x0() const               { return x0;     }
    Vector get_x() const                { return x;      }
    Vector get_offset() const           { return (x-x0); }

    virtual void run()
    {
        if (action!=NULL)
        {
            Vector wrench, force(3);

            // get the wrench at the end-effector
            action->getExtWrench(wrench);            
            force[0]=wrench[0];
            force[1]=wrench[1];
            force[2]=wrench[2];

            // yield a small displacement
            // iff the force overcomes the threshold
            if (norm(force)>forceThres)
            {
                Vector o;
                action->getPose(x,o);
                action->reachPose(x+gain*force,o0);
            }
        }
    }

    virtual void threadRelease()
    {
        if (action!=NULL)
            action->stopControl();
    }
};

class graspThread: public yarp::os::Thread {
protected:
    yarp::os::ResourceFinder *rf;
    std::string partUsed;
    std::string armToBeUsed;
    std::string name;

    ActionPrimitivesLayer3       *actionL;
    ActionPrimitivesLayer3       *actionR;
    ActionPrimitivesLayer3       *action;

    //opdbAccessClient              opdbClient;
    std::map<std::string,yarp::sig::Matrix>*            palmOrientations;

    yarp::os::BufferedPort<yarp::os::Bottle>          cmdPort;
    yarp::os::Port                          rpcPort;

    kinematicCalibrator           kinCalib;
    kinematicOffset               dOffsL;
    kinematicOffset               dOffsR;
    tableManager                  tableMan;

    yarp::sig::Vector graspOrienL,           graspOrienR;
    yarp::sig::Vector graspDispL,            graspDispR;
    yarp::sig::Vector graspReliefL,          graspReliefR;
    yarp::sig::Vector dLiftL,                dLiftR;
    yarp::sig::Vector dTouchL,               dTouchR;
    yarp::sig::Vector dTapL,                 dTapR;
    yarp::sig::Vector dPushL,                 dPushR;
    yarp::sig::Vector home_xL,               home_xR;
    double dropLengthL,           dropLengthR;
    double forceCalibTableThresL, forceCalibTableThresR;
    double forceCalibKinThresL,   forceCalibKinThresR;

    yarp::sig::Vector *graspOrien;
    yarp::sig::Vector *graspDisp;
    yarp::sig::Vector *graspRelief;
    yarp::sig::Vector *dLift;
    yarp::sig::Vector *dTouch;
    yarp::sig::Vector *dTap;
    yarp::sig::Vector *dPush;
    yarp::sig::Vector *home_x;
    double *dropLength;
    double *forceCalibKinThres;
    kinematicOffset *dOffs;

    double targetInRangeThres;    

    bool openPorts;
    bool firstRun;
    bool running;
    bool trackMode;
    bool use_opdb;

public:
    /**
    * costructor
    */
    graspThread();

    /**
    * costructor
    */
    graspThread(ResourceFinder& rf);

    /**
    * destructor
    */
    ~graspThread();

    bool threadInit();

    void threadRelease();

    /**
    * running cycle of the thread
    */
    void run();

    void onStop();

     /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    void getArmDependentOptions(yarp::os::Bottle &b, kinematicOffset &_dOffs,
                                yarp::sig::Vector &_gDisp, yarp::sig::Vector &_gRelief,
                                yarp::sig::Vector &_dLift, yarp::sig::Vector &_dTouch,
                                yarp::sig::Vector &_dTap, yarp::sig::Vector &_dPush, yarp::sig::Vector &_home_x, double &_dropLength,
                                double &_forceCalibTableThres, double &_forceCalibKinThres);
    
    virtual bool configure(yarp::os::ResourceFinder &rf);


    /**
    * gets the refreshing rate
    */
    virtual double getPeriod();

    /**
    * computes the palm orientations
    */
    void computePalmOrientations();

    /**
    * determines which one of the arms should be used
    */
    void useArm(const int arm);

    /**
    * goes back to home position
    */
    void goHome();

    /**
    * retrieves the target position and prepare the arm
    */
    Vector retrieveTargetAndPrepareArm(yarp::os::Bottle *b);

    /**
    * determines whether the target is in range
    */
    bool isTargetInRange(const yarp::sig::Vector &xd);

    /**
    * points the target
    */
    void point(const yarp::sig::Vector &xd);

    /**
    * touches the target
    */
    void touch(const yarp::sig::Vector &xd);

    /***
    * grasps the target
    */
    void grasp(const yarp::sig::Vector &xd);

    /**
    * taps the target
    */
    void tap(const yarp::sig::Vector &xd);

    /**
    * pushes the target
    */
    void push(const yarp::sig::Vector &xd);

    bool close();

    /**
    * sets the tracking mode
    */
    void setTrackingMode(const bool sw);

    /**
    * interrupt the thread
    */
    bool interruptModule();

    /**
    * respond function
    */
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);


};


#endif  //_GRASP_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
