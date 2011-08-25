// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * @file main.cpp
 * @brief main code for launching the angel2SAC
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
//#include <yarp/dev/IDebugInterface.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/DebugInterfaces.h>
#include <stdio.h>
#include <string>
#include <cstring>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

static int MAX_NUMBER_ACTIVATED = 1;

int main(int argc, char * argv[]) {
    bool debug_param_enabled = true;
    
    
    YARP_REGISTER_DEVICES(icubmod)
    Network yarp;

    Time::turboBoost(); 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("angel2SAC.ini");         //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);    

    string ctrlName = "local";
    string robotName=rf.check("robot",Value("icub")).asString().c_str();
    string partName=rf.check("part",Value("head")).asString().c_str();
    Property optHead("(device remote_controlboard)");
    PolyDriver  *drvHead;
    IEncoders   *encHead;
    string remoteHeadName="/"+robotName+"/"+partName;
    string localHeadName="/"+ctrlName+"/"+partName;
    optHead.put("remote",remoteHeadName.c_str());
    optHead.put("local",localHeadName.c_str());
    drvHead =new PolyDriver(optHead);
    if (!drvHead->open(optHead)) {
        
        return false;
    }

    if (!drvHead->isValid()) {
        fprintf(stdout,"Head device driver not available!\n");
        
        delete drvHead;
        return false;
    }
    drvHead->view(encHead);
    

    // ------------------------------------------------------------ 
    //IPositionControl  *posTorso;

    /*Property options;

    yarp::os::Network::init();
  
    IDebugInterface   *iDbg = NULL;
    //PolyDriver *debugDd;
    PolyDriver *partsdd[MAX_NUMBER_ACTIVATED];
    PolyDriver *debugdd[MAX_NUMBER_ACTIVATED];
    char *partsName[1];
    int n = 0;
  
    std::string portLocalName2;
    std::string robotPartPort= "/";
    robotPartPort += robotName.c_str();
    robotPartPort += "/";
    robotPartPort += partsName[n];
    
    std::string robotPartDebugPort= "/";
    robotPartDebugPort += robotName.c_str();
    robotPartDebugPort += "/debug/";
    robotPartDebugPort += partsName[n];
    
    //checking existence of the port
    int ind = 0;
    string portLocalName;
    portLocalName="/";
    portLocalName+=robotName.c_str();
    portLocalName+="/robotMotorGui";
    char tmp[80];
    sprintf(tmp, "%d", ind);
    portLocalName+=tmp;
    portLocalName+="/";
    portLocalName+=partsName[n];
    
    options.put("local", portLocalName.c_str());	
    options.put("device", "remote_controlboard");
    options.put("remote", robotPartPort.c_str());
    options.put("carrier", "udp");
    partsdd[n] = new PolyDriver(options);
    
    if (debug_param_enabled) {
        Property debugOptions;
        portLocalName2=portLocalName;
        // the following complex line of code performs a substring substitution (see example below)
        // "/icub/robotMotorGui2/right_arm" -> "/icub/robotMotorGui2/debug/right_arm"
        portLocalName2.replace(portLocalName2.find(partsName[n]),strlen(partsName[n]),std::string("debug/")+std::string(partsName[n]));
        debugOptions.put("local", portLocalName2.c_str());	
        debugOptions.put("device", "debugInterfaceClient");
        debugOptions.put("remote", robotPartPort.c_str());
        debugOptions.put("carrier", "udp");
        debugdd[n] = new PolyDriver(debugOptions);
        if(debugdd[n]->isValid() == false) {
            fprintf(stderr, "Problems opening the debug client \n");
        }	
    }
    else {
        debugdd[n]=0;
    }
    
    */
    return 0;
}



