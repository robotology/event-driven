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
 * Public License fo
 r more details
 */

/**
 * @file oInteractorThread.cpp
 * @brief Implementation of the thread (see header oiThread.h)
 */

#include <iCub/graspThread.h>
#include <iCub/objectInteractorThread.h>
#include <cstring>
#include <string>
#include <cassert>
#include <map>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define THRATE 10


graspThread::graspThread() {
    palmOrientations=new map<string, Matrix>;

    computePalmOrientations();
       
    graspOrienL.resize(4);     graspOrienR.resize(4);
    graspDispL.resize(4);      graspDispR.resize(3);
    graspReliefL.resize(4);    graspReliefR.resize(3);
    dLiftL.resize(3);          dLiftR.resize(3);
    dTouchL.resize(3);         dTouchR.resize(3);
    dTapL.resize(3);           dTapR.resize(3);
    dPushL.resize(3);          dPushR.resize(3);
    home_xL.resize(3);         home_xR.resize(3);

    graspOrienL=dcm2axis((*palmOrientations)["left_down"]);
    graspOrienR=dcm2axis((*palmOrientations)["right_down"]);

    // default values for arm-dependent quantities
    graspDispL[0]=0.0;         graspDispR[0]=0.0;
    graspDispL[1]=0.0;         graspDispR[1]=0.0; 
    graspDispL[2]=0.05;        graspDispR[2]=0.05;

    graspReliefL[0]=0.0;       graspReliefR[0]=0.0; 
    graspReliefL[1]=0.0;       graspReliefR[1]=0.0; 
    graspReliefL[2]=0.02;      graspReliefR[2]=0.02;

    dLiftL[0]=0.0;             dLiftR[0]=0.0;  
    dLiftL[1]=0.0;             dLiftR[1]=0.0;  
    dLiftL[2]=0.15;            dLiftR[2]=0.15; 
                                
    dTouchL[0]=0.0;            dTouchR[0]=0.0;
    dTouchL[1]=0.0;            dTouchR[1]=0.0;  
    dTouchL[2]=0.0;            dTouchR[2]=0.0;
                               
    dTapL[0]=0.0;              dTapR[0]=0.0;
    dTapL[1]=0.0;              dTapR[1]=0.0;  
    dTapL[2]=0.0;              dTapR[2]=0.0;

    dPushL[0]=0.0;              dPushR[0]=0.0;
    dPushL[1]=0.0;              dPushR[1]=0.0;
    dPushL[2]=0.0;              dPushR[2]=0.0;

    dropLengthL=0.0;           dropLengthR=0.0;
    forceCalibTableThresL=1e9; forceCalibTableThresR=1e9;
    forceCalibKinThresL=1e9;   forceCalibKinThresR=1e9;

    home_xL[0]=-0.29;          home_xR[0]=-0.29;
    home_xL[1]=-0.21;          home_xR[1]= 0.24;
    home_xL[2]= 0.11;          home_xR[2]= 0.07;

    action=actionL=actionR = NULL;
    graspOrien = NULL;
    graspDisp = NULL;
    graspRelief = NULL;
    dOffs = NULL;
    dLift = NULL;
    dTouch = NULL;
    dTap = NULL;
    dPush = NULL;
    home_x = NULL;
    dropLength = NULL;

    openPorts = false;
    firstRun = true;
    running = false;

    Rand::init();
}



graspThread::graspThread(ResourceFinder& rf) {
    
    palmOrientations=new map<string,Matrix>;
    computePalmOrientations();
       
    graspOrienL.resize(4);     graspOrienR.resize(4);
    graspDispL.resize(4);      graspDispR.resize(3);
    graspReliefL.resize(4);    graspReliefR.resize(3);
    dLiftL.resize(3);          dLiftR.resize(3);
    dTouchL.resize(3);         dTouchR.resize(3);
    dTapL.resize(3);           dTapR.resize(3);
    dPushL.resize(3);           dPushR.resize(3);
    home_xL.resize(3);         home_xR.resize(3);

    graspOrienL=dcm2axis((*palmOrientations)["left_down"]);
    graspOrienR=dcm2axis((*palmOrientations)["right_down"]);

    // default values for arm-dependent quantities
    graspDispL[0]=0.0;         graspDispR[0]=0.0;
    graspDispL[1]=0.0;         graspDispR[1]=0.0; 
    graspDispL[2]=0.05;        graspDispR[2]=0.05;

    graspReliefL[0]=0.0;       graspReliefR[0]=0.0; 
    graspReliefL[1]=0.0;       graspReliefR[1]=0.0; 
    graspReliefL[2]=0.02;      graspReliefR[2]=0.02;

    dLiftL[0]=0.0;             dLiftR[0]=0.0;  
    dLiftL[1]=0.0;             dLiftR[1]=0.0;  
    dLiftL[2]=0.15;            dLiftR[2]=0.15; 
                                
    dTouchL[0]=0.0;            dTouchR[0]=0.0;
    dTouchL[1]=0.0;            dTouchR[1]=0.0;  
    dTouchL[2]=0.0;            dTouchR[2]=0.0;
                               
    dTapL[0]=0.0;              dTapR[0]=0.0;
    dTapL[1]=0.0;              dTapR[1]=0.0;  
    dTapL[2]=0.0;              dTapR[2]=0.0;

    dPushL[0]=0.0;              dPushR[0]=0.0;
    dPushL[1]=0.0;              dPushR[1]=0.0;
    dPushL[2]=0.0;              dPushR[2]=0.0;

    dropLengthL=0.0;           dropLengthR=0.0;
    forceCalibTableThresL=1e9; forceCalibTableThresR=1e9;
    forceCalibKinThresL=1e9;   forceCalibKinThresR=1e9;

    home_xL[0]=-0.29;          home_xR[0]=-0.29;
    home_xL[1]=-0.21;          home_xR[1]= 0.24;
    home_xL[2]= 0.11;          home_xR[2]= 0.07;

    action=actionL=actionR = NULL;
    graspOrien = NULL;
    graspDisp = NULL;
    graspRelief = NULL;
    dOffs = NULL;
    dLift = NULL;
    dTouch = NULL;
    dTap = NULL;
    dPush = NULL;
    home_x = NULL;
    dropLength = NULL;

    openPorts = false;
    firstRun = true;
    running = false;

    Rand::init();

    configure(rf);
}



graspThread::~graspThread() {

}

bool graspThread::threadInit() {
    return true;
}

void graspThread::threadRelease() {

}

void graspThread::onStop() {
    close();
}

void graspThread::getArmDependentOptions(Bottle &b, kinematicOffset &_dOffs,
                                Vector &_gDisp, Vector &_gRelief,
                                Vector &_dLift, Vector &_dTouch,
                                Vector &_dTap, Vector &_dPush, Vector &_home_x, double &_dropLength,
                                double &_forceCalibTableThres, double &_forceCalibKinThres) {
    string kinOffsfileName=b.find("kinematic_offsets_file").asString().c_str();
    _dOffs.load(rf->findFile(kinOffsfileName.c_str()).c_str());

    if (Bottle *pB=b.find("grasp_displacement").asList())
    {
        int sz=pB->size();
        int len=_gDisp.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _gDisp[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("grasp_relief").asList())
    {
        int sz=pB->size();
        int len=_gRelief.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _gRelief[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("lifting_displacement").asList())
    {
        int sz=pB->size();
        int len=_dLift.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _dLift[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("touching_displacement").asList())
    {
        int sz=pB->size();
        int len=_dTouch.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _dTouch[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("tapping_displacement").asList())
    {
        int sz=pB->size();
        int len=_dTap.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _dTap[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("pushing_displacement").asList())
    {
        int sz=pB->size();
        int len=_dPush.length();
        int l=len<sz?len:sz;
        printf("pushing_displacement \n");
        for (int i=0; i<l; i++)
            _dPush[i]=pB->get(i).asDouble();
        printf("pushing_displacement \n");
    }

    if (Bottle *pB=b.find("home_position").asList())
    {
        int sz=pB->size();
        int len=_home_x.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _home_x[i]=pB->get(i).asDouble();
    }

    if (b.check("dropping_length"))
        _dropLength=b.find("dropping_length").asDouble();

    if (b.check("force_calib_table_thres"))
        _forceCalibTableThres=b.find("force_calib_table_thres").asDouble();

    if (b.check("force_calib_kin_thres"))
        _forceCalibKinThres=b.find("force_calib_kin_thres").asDouble();
}


void graspThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string graspThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}


bool graspThread::configure(ResourceFinder &rf) {
    this->rf=&rf;

    string name=rf.find("name").asString().c_str();
    setName(name.c_str());

    partUsed=rf.check("part",Value("both_arms")).asString().c_str();
    if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
    {
        cout<<"Invalid part requested !"<<endl;
        return false;
    }

    Property config; config.fromConfigFile(rf.findFile("from").c_str());
    Bottle &bGeneral=config.findGroup("general");
    if (bGeneral.isNull())
    {
        cout<<"Error: group general is missing!"<<endl;
        return false;
    }

    // parsing general config options
    Property option;
    for (int i=1; i<bGeneral.size(); i++)
    {
        Bottle *pB=bGeneral.get(i).asList();
        if (pB->size()==2)
            option.put(pB->get(0).asString().c_str(),pB->get(1));
        else
        {
            cout<<"Error: invalid option!"<<endl;
            return false;
        }
    }

    option.put("local",name.c_str());
    option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());

    Property optionL(option); optionL.put("part","left_arm");
    Property optionR(option); optionR.put("part","right_arm");

    // get tracking mode
    trackMode=(option.check("tracking_mode",Value("off")).asString()=="on");

    // get kinematic calibration options
    kinCalib.set_gain(option.check("calib_kin_gain",Value(0.0)).asDouble());

    // get in range threshold
    targetInRangeThres=option.check("target_in_range_thres",Value(1e9)).asDouble();

    // get table configuration
    tableMan.load(rf.findFile("table_file").c_str());

    // init table parameters transmission
    tableMan.initTxParams(("/"+name+"/table:o").c_str(),
                          rf.find("homography_port").asString().c_str());

    // parsing left_arm config options
    Bottle &bLeft=config.findGroup("left_arm");
    if (bLeft.isNull())
    {
        cout<<"Error: group left_arm is missing!"<<endl;
        return false;
    }
    else
        getArmDependentOptions(bLeft,dOffsL,graspDispL,graspReliefL,
                               dLiftL,dTouchL,dTapL,dPushL,home_xL,dropLengthL,
                               forceCalibTableThresL,forceCalibKinThresL);

    // parsing right_arm config options
    Bottle &bRight=config.findGroup("right_arm");
    if (bRight.isNull())
    {
        cout<<"Error: group right_arm is missing!"<<endl;
        return false;
    }
    else
        getArmDependentOptions(bRight,dOffsR,graspDispR,graspReliefR,
                               dLiftR,dTouchR,dTapR,dPushR,home_xR,dropLengthR,
                               forceCalibTableThresR,forceCalibKinThresR);

    // set up the reaching timeout
    double reachingTimeout=2.0 * option.check("default_exec_time",Value("3.0")).asDouble();

    if (partUsed=="both_arms" || partUsed=="left_arm")
    {
        cout<<"***** Instantiating primitives for left_arm"<<endl;
        actionL=new ActionPrimitivesLayer2(optionL);
        actionL->setExtForceThres(forceCalibTableThresL);
        actionL->enableReachingTimeout(reachingTimeout);

        if (!actionL->isValid())
        {
            printf("left Action is not valid! \n");
            close();
            return false;
        }
        else
            useArm(USE_LEFT);
    }

    if (partUsed=="both_arms" || partUsed=="right_arm")
    {
        cout<<"***** Instantiating primitives for right_arm"<<endl;
        actionR=new ActionPrimitivesLayer2(optionR);
        actionR->setExtForceThres(forceCalibTableThresR);
        actionR->enableReachingTimeout(reachingTimeout);

        if (!actionR->isValid())
        {
            close();
            return false;
        }
        else
            useArm(USE_RIGHT);
    }        

    // access the opdb to retrieve the close_hand sequence
    /*
    use_opdb=(option.check("use_opdb",Value("off")).asString()=="on");
    if (use_opdb)
    {          
        string serverName=rf.find("opdbServerName").asString().c_str();
        string clientName="/"+name+serverName;
        if (!opdbClient.open(clientName,serverName))
        {
            cout<<"Error: unable to access OPDB!"<<endl;

            close();
            return false;
        }

        Bottle closeHandReq;
        Bottle closeHandImpl;

        closeHandReq.addString("GraspImpl");

        if (opdbClient.getGraspData(0,"ICUB","CloseHand",closeHandReq,closeHandImpl)==CHRIS_OK)
        {
            // remove quotes from the string
            string tmp=closeHandImpl.toString().c_str();
            tmp=tmp.substr(1,tmp.length()-2);
            Bottle closeHandSeq(tmp.c_str());

            if (actionR)
            {
                actionR->removeHandSeq("close_hand");
                if (!actionR->addHandSequence("close_hand",closeHandSeq))
                {
                    cout<<"Error: incorrect params retrieved from OPDB!"<<endl;

                    close();
                    return false;
                }
            }

            if (actionL)
            {
                actionL->removeHandSeq("close_hand");
                if (!actionL->addHandSequence("close_hand",closeHandSeq))
                {
                    cout<<"Error: incorrect params retrieved from OPDB!"<<endl;

                    close();
                    return false;
                }
            }
        }
        else
        {
            cout<<"Error: unable to retrieve params from OPDB!"<<endl;

            close();
            return false;
        }
    }
    */

    // print out the hand sequences
    deque<string> q=action->getHandSeqList();
    cout<<"***** List of available hand sequence keys:"<<endl;
    for (size_t i=0; i<q.size(); i++)
    {
        Bottle sequence;
        action->getHandSequence(q[i],sequence);

        cout<<"***** "<<q[i]<<":"<<endl;
        cout<<sequence.toString()<<endl;
    }

    cmdPort.open(("/"+name+"/cmd:i").c_str());
    rpcPort.open(("/"+name+"/rpc").c_str());
    //attach(rpcPort);

    openPorts=true;

    return true;
}

bool graspThread::close()
{
    //opdbClient.close();

    if (actionL!=NULL)
        delete actionL;

    if (actionR!=NULL)
        delete actionR;        

    if (openPorts)
    {
        cmdPort.close();
        rpcPort.close();
    }

    return true;
}

double graspThread::getPeriod()
{
    return 0.1;
}

void graspThread::computePalmOrientations() {
    Matrix Ry(3,3);
    Matrix R(3,3);
    Ry.zero();
    Ry(0,0)=cos(M_PI);
    Ry(0,2)=sin(M_PI);
    Ry(1,1)=1.0;
    Ry(2,0)=-Ry(0,2);
    Ry(2,2)=Ry(0,0);
    map<string,Matrix>::iterator it;
    
    palmOrientations->insert( pair<string,Matrix>("right_down",Ry));
    it = palmOrientations->begin();
    //palmOrientations["right_down"]=Ry;
    
    Matrix Rx(3,3);
    Rx.zero();
    Rx(0,0)=1.0;
    Rx(1,1)=cos(M_PI);
    Rx(1,2)=-sin(M_PI);
    Rx(2,1)=-Rx(1,2);
    Rx(2,2)=Rx(1,1);
    
    palmOrientations->insert(it, pair<string,Matrix>("left_down",Ry * Rx));
    //palmOrientations["left_down"]=Ry*Rx;
    
    Rx(1,1)=cos(M_PI/2.0);
    Rx(1,2)=-sin(M_PI/2.0);
    Rx(2,1)=-Rx(1,2);
    Rx(2,2)=Rx(1,1);
    R = (*palmOrientations)["right_down"] * Rx;
    palmOrientations->insert(it, pair<string,Matrix>("right_base",R));
    //palmOrientations["right_base"]=palmOrientations["right_down"]*Rx;
    
    Rx(1,1)=cos(-M_PI/2.0);
    Rx(1,2)=-sin(-M_PI/2.0);
    Rx(2,1)=-Rx(1,2);
    Rx(2,2)=Rx(1,1);
    R = (*palmOrientations)["left_down"] * Rx;
    palmOrientations->insert(it, pair<string,Matrix>("left_base",R));
    //palmOrientations["left_base"]=palmOrientations["left_down"]*Rx;
    
    Matrix Rz(3,3);
    Rz.zero();
    Rz(0,0)=cos(M_PI/4.0);
    Rz(0,1)=-sin(M_PI/4.0);
    Rz(1,0)=-Rz(0,1);
    Rz(1,1)=Rz(0,0);
    Rz(2,2)=1.0;
    R = (*palmOrientations)["right_base"];
    palmOrientations->insert(it, pair<string,Matrix>("right_starttap",R));
    //palmOrientations["right_starttap"] = palmOrientations["right_base"];
    R = (*palmOrientations)["left_base"];
    palmOrientations->insert(it, pair<string,Matrix>("left_starttap",R));
    //palmOrientations["left_starttap"] = palmOrientations["left_base"];
    
    Rx(1,1)=cos(M_PI/8.0);
    Rx(1,2)=-sin(M_PI/8.0);
    Rx(2,1)=-Rx(1,2);
    Rx(2,2)=Rx(1,1);
    R = (*palmOrientations)["right_starttap"];
    palmOrientations->insert(it, pair<string,Matrix>("right_stoptap",R));
    //palmOrientations["right_stoptap"]=palmOrientations["right_starttap"];
    
    Rx(1,1)=cos(-M_PI/8.0);
    Rx(1,2)=-sin(-M_PI/8.0);
    Rx(2,1)=-Rx(1,2);
    Rx(2,2)=Rx(1,1);
    R = (*palmOrientations)["left_starttap"];
    palmOrientations->insert(it, pair<string,Matrix>("left_stoptap",R));
    //palmOrientations["left_stoptap"]=palmOrientations["left_starttap"];

    Ry(0,0)=cos(-M_PI / 2);
    Ry(0,2)=sin(-M_PI / 2);
    Ry(1,1)=1.0;
    Ry(2,0)=-Ry(0,2);
    Ry(2,2)=Ry(0,0);
    R = (*palmOrientations)["left_down"] * Ry;
    palmOrientations->insert(it, pair<string,Matrix>("left_startpush",R));
    palmOrientations->insert(it, pair<string,Matrix>("left_stoppush",R));

    Ry(0,0)=cos(M_PI / 2);
    Ry(0,2)=sin(M_PI / 2);
    Ry(1,1)=1.0;
    Ry(2,0)=-Ry(0,2);
    Ry(2,2)=Ry(0,0);
    R = (*palmOrientations)["right_down"] * Ry;
    palmOrientations->insert(it, pair<string,Matrix>("right_startpush",R));
    palmOrientations->insert(it, pair<string,Matrix>("right_stoppush",R));
}

void graspThread::useArm(const int arm) {
    if (arm==USE_LEFT)
    {
        action=actionL;
        armToBeUsed="left";

        graspOrien=&graspOrienL;
        graspDisp=&graspDispL;
        graspRelief=&graspReliefL;
        dOffs=&dOffsL;
        dLift=&dLiftL;
        dTouch=&dTouchL;
        dTap = &dTapL;
        dPush = &dPushL;
        home_x=&home_xL;
        dropLength=&dropLengthL;
        forceCalibKinThres=&forceCalibKinThresL;
    }
    else if (arm==USE_RIGHT)
    {
        action=actionR;
        armToBeUsed="right";

        graspOrien=&graspOrienR;
        graspDisp=&graspDispR;
        graspRelief=&graspReliefR;
        dOffs=&dOffsR;
        dLift=&dLiftR;
        dTouch=&dTouchR;
        dTap=&dTapR;
        dPush = &dPushR;
        home_x=&home_xR;
        dropLength=&dropLengthR;
        forceCalibKinThres=&forceCalibKinThresR;
    }
}

void graspThread::goHome() {
    bool f;

    // regardless of the current settings
    // go home in tracking mode
    bool latchTrackMode=trackMode;
    setTrackingMode(true);

    if (partUsed=="both_arms" || partUsed=="right_arm")
    {
        useArm(USE_RIGHT);
        action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }

    if (partUsed=="both_arms" || partUsed=="left_arm")
    {
        useArm(USE_LEFT);
        action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }

    setTrackingMode(latchTrackMode);
}

Vector graspThread::retrieveTargetAndPrepareArm(Bottle *b) {
    Vector xd(3);
    xd[0]=b->get(1).asDouble();
    xd[1]=b->get(2).asDouble();
    xd[2]=b->get(3).asDouble();

    // switch only if it's allowed
    if (partUsed=="both_arms")
    {
        if (xd[1]>0.0)
            useArm(USE_RIGHT);
        else
            useArm(USE_LEFT);
    }

    // apply systematic offset
    // due to uncalibrated kinematic
    Vector offs=dOffs->get(xd);
    xd[0]+=offs[0];
    xd[1]+=offs[1];

    // maintain the height in order
    // to use contact detection

    // distance safe thresholding
    xd[0]=xd[0]>-0.1?-0.1:xd[0];

    return xd;
}

bool graspThread::isTargetInRange(const Vector &xd) {
    if (norm(xd) < targetInRangeThres)
        return true;
    else
        return false;
}

void graspThread::point(const Vector &xd) {
    bool f=false;

    action->latchWrenchOffset();
    action->enableContactDetection();
    action->pushWaitState(1.0);
    action->pushAction(xd,*graspOrien,"open_hand");
    action->checkActionsDone(f,true);
    action->pushWaitState(2.0);
    action->disableContactDetection();
    action->pushAction(*home_x,HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

void graspThread::touch(const Vector &xd) {
    bool f=false;
    action->touch(xd,*graspOrien,*dTouch);
    action->pushWaitState(2.0);
    action->pushAction(xd+*dTouch,*graspOrien,HOMING_PERIOD);
    action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

void graspThread::grasp(const Vector &xd) {
    bool f=false;

    action->grasp(xd,*graspOrien,*graspDisp,*graspRelief);
    action->checkActionsDone(f,true);
    action->areFingersInPosition(f);

    // if fingers are not in position then
    // this means that we have hopefully grabbed
    // the object
    if (!f)
    {
        Vector randOffs=*dropLength*Rand::vector(3);
        randOffs[0]-=*dropLength/2.0;
        randOffs[1]-=*dropLength/2.0;
        randOffs[2]=0.0;

        action->pushAction(xd+*dLift,*graspOrien);
        action->pushWaitState(2.0);
        action->checkActionsDone(f,true);
        action->latchWrenchOffset();
        action->enableContactDetection();
        action->pushWaitState(1.0);
        action->pushAction(xd+randOffs,*graspOrien,3.0);
        action->checkActionsDone(f,true);
        action->disableContactDetection();

        Vector x,o;
        action->getPose(x,o);
        x[2]+=0.02;
        action->pushAction(x,*graspOrien);
        action->pushWaitState(2.0);
    }

    action->pushAction("release_hand");
    action->pushWaitState(2.0);
    action->pushAction(xd+*graspDisp,*graspOrien,HOMING_PERIOD);
    action->pushAction(*home_x,HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

void graspThread::tap(const Vector &xd) {
    bool f=false;

    Vector startPos=xd+*dTap;
    Vector endPos=startPos;
    endPos[1]-=2.0*(*dTap)[1];

    Vector startOrientation=dcm2axis((*palmOrientations)[armToBeUsed+"_starttap"]);
    Vector stopOrientation=dcm2axis((*palmOrientations)[armToBeUsed+"_stoptap"]);
    action->tap(startPos,startOrientation,endPos,stopOrientation,3.0);
    action->pushWaitState(2.0);
    action->pushAction(*home_x,dcm2axis((*palmOrientations)[armToBeUsed+"_down"]),"open_hand",HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

void graspThread::push(const Vector &xd) {
    bool f = false;

    Vector startPos = xd + *dPush;
    Vector endPos = startPos;
    endPos[1] -= 2.0 * (*dPush)[0];

    Vector startOrientation = dcm2axis((*palmOrientations)[armToBeUsed+"_startpush"]);
    Vector stopOrientation = dcm2axis((*palmOrientations)[armToBeUsed+"_stoppush"]);
    action->tap(startPos,startOrientation,endPos,stopOrientation,3.0);
    action->pushWaitState(2.0);
    action->pushAction(*home_x, dcm2axis((*palmOrientations)[armToBeUsed+"_down"]), "open_hand", HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

// we don't need a thread since the actions library already
// incapsulates one inside dealing with all the tight time constraints
void graspThread::run() {
    while (isStopping() != true) {
        printf("action %0x",action);
        // do it only once
        if (firstRun) {
            goHome();
            firstRun=false;
        }

        // get a target object position from a YARP port
        Bottle *b=cmdPort.read();    // blocking call

        if (b!=NULL) {
            int cmd = b->get(0).asVocab();

            switch (cmd) {
                // execute a touch
                case CMD_TOUCH: {
                    Vector xd=retrieveTargetAndPrepareArm(b);
                    running=true;
                    if (isTargetInRange(xd))
                        touch(xd);
                    else {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }
                    running=false;
                    break;
                }

                // execute a grasp
                case CMD_GRASP: {
                    Vector xd=retrieveTargetAndPrepareArm(b);

                    running=true;

                    if (isTargetInRange(xd))
                        grasp(xd);
                    else {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }

                    running=false;
                    break;
                }

                // execute a tap
                case CMD_TAP: {
                    Vector xd=retrieveTargetAndPrepareArm(b);
    
                    running=true;
    
                    if (isTargetInRange(xd))
                        tap(xd);
                    else {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }
                    running=false;
                    break;
                }

                              // execute a tap
                case CMD_PUSH: {
                    Vector xd = retrieveTargetAndPrepareArm(b);
    
                    running = true;
    
                    if (isTargetInRange(xd))
                        push(xd);
                    else {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }
                    running = false;
                    break;
                }

                // perform calibration
                case CMD_CALIB: {
                    int kind=b->get(1).asVocab();

                    // find out the table height
                    if (kind==OPT_TABLE) {
                        Vector x0(3), x1(3);
                        x0[0]=x1[0]=Rand::scalar(-0.35,-0.30);
                        x0[1]=x1[1]=Rand::scalar(-0.05,0.05);
                        x0[2]=0.1; x1[2]=-0.2;

                        // switch only if it's allowed
                        if (partUsed=="both_arms") {
                            if (x1[1]>0.0)
                                useArm(USE_RIGHT);
                            else
                                useArm(USE_LEFT);
                        }

                        running=true;
                        bool f=false;
    
                        action->pushAction(x0,*graspOrien,"open_hand");
                        action->checkActionsDone(f,true);
                        action->latchWrenchOffset();
                        action->enableContactDetection();
                        action->pushWaitState(1.0);
                        action->pushAction(x1,*graspOrien,3.0);
                        action->checkActionsDone(f,true);
                        action->pushWaitState(2.0);
                        action->disableContactDetection();
                        action->checkContact(f);

                        if (f) {
                            Vector x,o;
                            action->getPose(x,o);
                            double tableHeight=x[2];

                            cout<<"########## Table height found = "<<tableHeight<<endl;
                            tableMan.setTableHeight(tableHeight);
                        }
                        else
                            cout<<"########## Table not found"<<endl;

                        action->pushAction(x0,*graspOrien,HOMING_PERIOD);
                        action->pushAction(*home_x,HOMING_PERIOD);
                        action->checkActionsDone(f,true);
                        goHome();

                        running=false;
                    }
                    // kinematic offset calibration
                    else if (kind==OPT_KIN) {
                        int subcmd=b->get(2).asVocab();

                        // start part
                        if (subcmd==OPT_START) {
                            int type=b->get(3).asVocab();

                            Vector xd(3);
                            xd[0]=b->get(4).asDouble();
                            xd[1]=b->get(5).asDouble();
                            xd[2]=b->get(6).asDouble();
    
                            // switch only if it's allowed
                            if (partUsed=="both_arms") {
                                if (type==OPT_RIGHT)
                                    useArm(USE_RIGHT);
                                else
                                    useArm(USE_LEFT);
                            }

                            running=true;
                            bool f=false;

                            // apply systematic offset
                            // due to uncalibrated kinematic
                            Vector offs=dOffs->get(xd);
                            offs[2]=0.0;

                            action->pushAction(xd+offs+*graspDisp,*graspOrien,"open_hand");
                            action->checkActionsDone(f,true);
                            action->latchWrenchOffset();
                            action->enableContactDetection();
                            action->pushWaitState(1.0);
                            action->pushAction(xd+offs,*graspOrien,3.0);
                            action->checkActionsDone(f,true);
                            action->disableContactDetection();

                            kinCalib.init(action,xd,*forceCalibKinThres);

                            if (kinCalib.isRunning())
                                kinCalib.resume();
                            else
                                kinCalib.start();
                        }
                        // stop part
                        else if (subcmd==OPT_STOP) {
                            kinCalib.suspend();

                            // update kinematic offsets map
                            dOffs->insert(kinCalib.get_x0(),kinCalib.get_offset());

                            Vector xd=kinCalib.get_x();
                            xd[2]=kinCalib.get_x0()[2]; // keep the original height
                            grasp(xd);
                            running=false;
                        }
                        // clear part
                        else if (subcmd==OPT_CLEAR) {
                            int type=b->get(3).asVocab();

                            if (type==OPT_RIGHT)
                                dOffsR.clear();
                            else
                                dOffsL.clear();
                        }
                    }

                    break;
                }

                default: {
                    cout<<"Error: command not recognized!"<<endl;
                    break;
                }
            }
        }
    }
    return;
}

void graspThread::setTrackingMode(const bool sw) {
    if (partUsed=="both_arms" || partUsed=="left_arm")
        actionL->setTrackingMode(sw);

    if (partUsed=="both_arms" || partUsed=="right_arm")
        actionR->setTrackingMode(sw);

    trackMode=sw;
}

bool graspThread::interruptModule() {
    // since a call to checkActionsDone() blocks
    // the execution until it's done, we need to 
    // take control and exit from the waiting state
    action->syncCheckInterrupt(true);        

    cmdPort.interrupt();
    rpcPort.interrupt();

    if (kinCalib.isRunning())
        kinCalib.stop();

    return true;
}

bool graspThread::respond(const Bottle &command, Bottle &reply) {
    switch (command.get(0).asVocab()) {
        case VOCAB4('s','t','a','t'): {
            reply.addInt((int)running);
            return true;
        }

        case VOCAB4('t','r','a','c'): {
            if (command.size()>1)
            {
                bool sw=(command.get(1).asVocab()==OPT_ON);
                setTrackingMode(sw);
            }
        }

        //default:
        //    return RFModule::respond(command,reply);
    }

    return true;
}

