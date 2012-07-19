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
    if (configFile=="") {
        printf("ERROR: file not found");
        return false;
    }
    
    //---------------------------------------------------------------------------------

    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::sig::Matrix *invPrjL, *invPrjR;           // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;                 // projection matrix
    yarp::os::Property optionsHead;
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    int originalContext;                            // original context for the gaze Controller
    int cxl,cyl;                                    // center of the eye in the configfile
    double blockNeckPitchValue = -1;
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
        //eyeChain->rmLink(7);
        //eyeChain->rmLink(6); ;
        iKinLink* link = &(eyeChain-> operator ()(5));
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //double a_value = link->getA();
        //printf("a value %f \n", a_value);
        link->setD(0.145);
        link = &(eyeChain-> operator ()(6));
        link->setD(0.0);
        //eyeChain->blockLink(6,0.0);
        //eyeChain->blockLink(7,0.0);
        //link = &(eyeChain-> operator ()(6));
        //link->setA(0.0);
        //link->setD(0.034);
        //link->setAlpha(0.0);
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //iKinLink twistLink(0.0,0.034,M_PI/2.0,0.0,-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD);
        //*eyeChain << twistLink;
        //eyeL->releaseLink(6);

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

    //--------------------------------------------------------------------
    

   
    
    return 0;
}




