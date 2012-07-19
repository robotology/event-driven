// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file main.cpp
 * @brief main code for launching the test case for wings
 */
 
#define TmpDiff128AngelX_SIZE 128
#define TmpDiff128AngelY_SIZE 128
#define X_SIZE 128
#define Y_SIZE 128
#define MODEA 1
#define MODEB 3
#define MODEC 5
#define MODED 7

//#define TOMAPPER

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>

#include <iCub/DebugInterfaces.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <stdio.h>
#include <string>
#include <cstring>
#include <cstdlib>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace yarp::math;
using namespace std;

static int MAX_NUMBER_ACTIVATED = 1;

bool project(yarp::dev::IEncoders *encTorso,yarp::dev::IEncoders *encHead,
yarp::sig::Matrix *invPrjL,iCub::iKin::iCubEye *eyeL,int u, int v, double varDistance, Vector &xo) {
    Vector fp(3);
    
    Vector torso(3);
    encTorso->getEncoder(0,&torso[0]);
    encTorso->getEncoder(1,&torso[1]);
    encTorso->getEncoder(2,&torso[2]);
    Vector head(5);
    encHead->getEncoder(0,&head[0]);
    encHead->getEncoder(1,&head[1]);
    encHead->getEncoder(2,&head[2]);
    encHead->getEncoder(3,&head[3]);
    encHead->getEncoder(4,&head[4]);
    
    
    Vector q(8);
    double ratio = M_PI /180;
    q[0]=torso[0] * ratio;
    q[1]=torso[1]* ratio;
    q[2]=torso[2]* ratio;
    q[3]=head[0]* ratio;
    q[4]=head[1]* ratio;
    q[5]=head[2]* ratio;
    q[6]=head[3]* ratio;
    q[7]=head[4]* ratio;
    double ver = head[5];                    
    
    Vector x(3);
    printf("varDistance %f \n", varDistance);
    x[0]=varDistance * u;   //epipolar correction excluded the focal lenght
    x[1]=varDistance * v;
    x[2]=varDistance;
    
    // find the 3D position from the 2D projection,
    // knowing the distance z from the camera
    Vector xe = yarp::math::operator *(*invPrjL, x);
    xe[3]=1.0;  // impose homogeneous coordinates                
    
    // update position wrt the root frame
    Matrix eyeH = eyeL->getH(q);
    //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
    xo = yarp::math::operator *(eyeH,xe);
    
    printf("object %f,%f,%f \n",xo[0],xo[1],xo[2]);    

    return true;
}

/*
bool projectPoint(const string &type, const double u, const double v, const double z, Vector &x) {
    bool isLeft=(type=="left");

    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (invPrj)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        Vector q(8);
        q[0]=torso[0];
        q[1]=torso[1];
        q[2]=torso[2];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];

        if (isLeft)
            q[7]=head[4]+head[5]/2.0;
        else
            q[7]=head[4]-head[5]/2.0;

        Vector p(3);
        p[0]=z*u;
        p[1]=z*v;
        p[2]=z;

        // find the 3D position from the 2D projection,
        // knowing the coordinate z in the camera frame
        Vector xe=*invPrj*p;
        xe[3]=1.0;  // impose homogeneous coordinates

        // find position wrt the root frame
        mutex.wait();
        x=(eye->getH(q)*xe).subVector(0,2);
        mutex.post();

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",type.c_str());
        return false;
    }
}
*/

/************************************************************************/

bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                //double cx = parType.find("w").asDouble() / 2.0;
                //double cy = parType.find("h").asDouble() / 2.0;
                // we suppose that the centerof ditortion is NOT    compensated
                double cx = parType.find("cx").asDouble();
                double cy = parType.find("cy").asDouble();

                double fx = parType.find("fx").asDouble();
                double fy = parType.find("fy").asDouble();

                Matrix K  = eye(3,3);
                Matrix Pi = zeros(3,4);

                K(0,0) = fx;
                K(1,1) = fy;
                K(0,2) = cx;
                K(1,2) = cy; 
                
                Pi(0,0) = Pi(1,1) = Pi(2,2) = 1.0;

                *Prj = new Matrix;
                **Prj = K * Pi;

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}

/**********************************************************************************/

int main(int argc, char * argv[]) {
    bool debug_param_enabled = true;

    // initialize random seed: 
    srand ( time(NULL) );
    
    YARP_REGISTER_DEVICES(icubmod)
    //Network yarp;
    yarp::os::Network::init();
    Time::turboBoost();

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("testWings.ini");         //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);    

    yarp::os::ConstString configName             = rf.check("config", 
                           Value("icubEyes.ini"), 
                           "Config file for intrinsic parameters (string)").asString();
    printf("configFile: %s \n", configName.c_str());
    std::string configFile = (std::string) rf.findFile(configName.c_str());
    printf("config file %s \n", configFile.c_str());
    if (configFile == "") {
        printf("ERROR: file not found \n");
        return false;
    }

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    std::string robot               = (std::string)rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();

    
    
    //---------------------------------------------------------------------------------

    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::sig::Matrix *invPrjL, *invPrjR;           // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;                 // projection matrix
    yarp::os::Property optionsHead;                 // head options
    yarp::dev::IEncoders *encTorso, *encHead;       // measure of the encoder  (head and torso)
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    yarp::dev::PolyDriver *polyTorso, *robotHead;   // polydriver for the control of the head
    int originalContext;                            // original context for the gaze Controller
    int cxl,cyl;                                    // center of the eye in the configfile
    double blockNeckPitchValue = -1;
    double varDistance = 0.5;                       // distance from the element                 
    bool isOnWings = true;
    
    //--------------------------------------------------------------------------------
    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append("testWings");
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;
   
    igaze->storeContext(&originalContext);
  
    if(blockNeckPitchValue != -1) {
        igaze->blockNeckPitch(blockNeckPitchValue);
        printf("pitch fixed at %f \n",blockNeckPitchValue);
    }
    else {
        printf("pitch free to change\n");
    }

    // -----------------------------------------------------------------

    Bottle info;
    igaze->getInfo(info);
    printf("just got the info \n");    
    int head_version = info.check("head_version", Value(1)).asInt();
    
    printf("head_version extracted from gazeArbiter \n");

    if(head_version == 1) {
        eyeL = new iCubEye("left");
        eyeR = new iCubEye("right");
    }
    else {
        eyeL = new iCubEye("left_v2");
        eyeR = new iCubEye("right_v2");
    }
    printf("correctly istantiated the head \n");

    
    // remove constraints on the links
    // we use the chains for logging purpose
    // eyeL->setAllConstraints(false);
    // eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);

    // if it isOnWings, move the eyes on top of the head 
    if (isOnWings) {
        printf("changing the structure of the chain \n");
        iKinChain* eyeChain = eyeL->asChain();
        eyeChain->rmLink(7);
        eyeChain->rmLink(6); 
        eyeChain->rmLink(5);
        eyeChain->rmLink(4);
        eyeChain->rmLink(3);
        eyeChain->rmLink(2);
        eyeChain->rmLink(1);
        eyeChain->rmLink(0);

        iKinLink ikl0(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD);
        iKinLink ikl1(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD);
        iKinLink ikl2(0.00231, -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD);
        iKinLink ikl3(  0.033,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD);
        iKinLink ikl4(    0.0,   0.001, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD);
        iKinLink ikl5( -0.054,  0.0825, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD);
        iKinLink ikl6(    0.0,   0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD);
        iKinLink ikl7(    0.0,     0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD);

        eyeChain->pushLink(ikl0);
        eyeChain->pushLink(ikl1);
        eyeChain->pushLink(ikl2);
        eyeChain->pushLink(ikl3);
        eyeChain->pushLink(ikl4);
        eyeChain->pushLink(ikl5);
        eyeChain->pushLink(ikl6);
        eyeChain->pushLink(ikl7);

    }
    else {
        printf("isOnWing false \n");
    }

    // get camera projection matrix from the configFile
    printf("get Camera configuration \n");
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        cxl=Prj(0,2);
        cyl=Prj(1,2);
        printf("pixel fovea in the config file %d %d \n", cxl,cyl);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    // ------------------------------------------------------------------


    string headPort = "/" + robot + "/head";
    string nameLocal("gazeArbiter");

    //initialising the head polydriver
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", ("/"+nameLocal+"/localhead").c_str());
    optionsHead.put("remote", headPort.c_str());
    robotHead = new PolyDriver (optionsHead);

    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    robotHead->view(encHead);
    
    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    optPolyTorso.put("remote",("/"+robot+"/torso").c_str());
    optPolyTorso.put("local",("/"+nameLocal+"/torso/position").c_str());
    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
        return false;
    }
    polyTorso->view(encTorso);
    

    //--------------------------------------------------------------------
    int operation = 0;
    switch (operation) {

    case 0 : {
        Vector xo;
        //calculating the 3d position and sending it to database
        int u = 160; 
        int v = 120;
        varDistance  = 0.5;
        project(encTorso,encHead,invPrjL,eyeL, u,v, varDistance, xo);
        
    }break;
    case 1: {
        
       Vector px(2);   // specify the pixel where to look
       px[0]=160.0;
       px[1]=120.0;
       
       int u = 160;
       int v = 120;

       Vector plane(4);  // specify the plane in the root reference frame as ax+by+cz+d=0; z=-0.12 in this case
       plane[0]=0.0;     // a
       plane[1]=0.0;     // b
       plane[2]=1.0;     // c
       plane[3]=0.12;    // d
       
       Vector x;
 
        if (plane.length()<4)
            {
                fprintf(stdout,"Not enough values given for the projection plane!\n");
                return false;
            }
        
        ConstString type = "left";
        bool isLeft=(type=="left");
        iCubEye *eye=(isLeft?eyeL:eyeR);
        
        
        //if (projectPoint(type,u,v,1.0,x))
        if(project(encTorso,encHead,invPrjL,eyeL,u,v,1.0,x))
            {
                // pick up a point belonging to the plane
                Vector p0(3,0.0);
                if (plane[0]!=0.0)
                    p0[0]=-plane[3]/plane[0];
                else if (plane[1]!=0.0)
                    p0[1]=-plane[3]/plane[1];
                else if (plane[2]!=0.0)
                    p0[2]=-plane[3]/plane[2];
                else
                    {
                        fprintf(stdout,"Error while specifying projection plane!\n");
                        return false;
                    }
                
                // take a vector orthogonal to the plane
                Vector n(3);
                n[0]=plane[0];
                n[1]=plane[1];
                n[2]=plane[2];
                
                //mutex.wait();
                Vector e = eye->EndEffPose().subVector(0,2);
                //mutex.post();

                // compute the projection
                Vector v=x-e;
                x=e+(dot(p0-e,n)/dot(v,n))*v;
                
                return true;
            }
        else
            return false;
    }
        
    return 0;

    }
}




