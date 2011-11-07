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
        //Network yarp;
    yarp::os::Network::init();
    Time::turboBoost(); 
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("angel2SAC.ini");         //overridden by --from parameter
    rf.setDefaultContext("eMorphApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);    

    string ctrlName = "microsacTest";
    string robotName=rf.check("robot",Value("icub")).asString().c_str();
    string partName=rf.check("part",Value("head")).asString().c_str();
    Property optHead("(device remote_controlboard)");
    PolyDriver  *drvHead;
    IEncoders   *encHead;
    string remoteHeadName = "/"+robotName+"/"+partName;
    string localHeadName  = "/"+ctrlName+"/"+partName;
    optHead.put("remote",remoteHeadName.c_str());
    optHead.put("device", "remote_controlboard");
    optHead.put("local",localHeadName.c_str());
    drvHead =new PolyDriver(optHead);
    bool res = drvHead->open(optHead);
    if (res) {
        printf("Error in opening the optHead %d \n", res);
        //return false;
    }
    else {
        printf("Successfully opened the polydriver optHead \n");
    }

    if (!drvHead->isValid()) {
        fprintf(stdout,"Head device driver not available!\n");      
        printf("Head device driver not available! \n");      
        delete drvHead;
        return false;
    }
    else {
        printf("safely asserting that the drvHead is valid \n");
    }
    drvHead->view(encHead);
    
    
    
    //------------------------------------------------ 
    //IPositionControl  *posTorso;
    
    Property options;
    IDebugInterface   *iDbg = NULL;
    //PolyDriver *debugDd;
    PolyDriver *partsdd[MAX_NUMBER_ACTIVATED];
    PolyDriver *debugdd[MAX_NUMBER_ACTIVATED];
    
    int n = 0;
    //string pName("head");
    //partsName[0] = (const) pName.c_str();
  
    std::string portLocalName2;
    std::string robotPartPort= "/";
    robotPartPort += robotName.c_str();
    robotPartPort += "/";
    robotPartPort += partName;
    
    std::string robotPartDebugPort= "/";
    robotPartDebugPort += robotName.c_str();
    robotPartDebugPort += "/debug/";
    robotPartDebugPort += partName;
    
    //checking existence of the port
    int ind = 0;
    string portLocalName;
    portLocalName="/";
    portLocalName+=robotName.c_str();
    portLocalName+="/microsaccTest";
    char tmp[80];
    sprintf(tmp, "%d", ind);
    portLocalName+=tmp;
    portLocalName+="/";
    portLocalName+=partName;
    
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
        portLocalName2.replace(portLocalName2.find(partName),
                               strlen(partName.c_str()),
                               std::string("debug/")+std::string(partName));
        debugOptions.put("local", portLocalName2.c_str());	
        debugOptions.put("device", "debugInterfaceClient");
        debugOptions.put("remote", robotPartPort.c_str());
        debugOptions.put("carrier", "udp");
        debugdd[n] = new PolyDriver(debugOptions);
        if(debugdd[n]->isValid() == false) {
            fprintf(stderr, "Problems opening the debug client \n");
            printf("Problems opening the debug client \n");
        }
        else {
            printf("safely asserting that the debug is valid \n");
        }
    }
    else {
        debugdd[n]=0;
    }
    
    debugdd[n]->view(iDbg);
    double dtf3 = 0;
    double dtf4 = 0;
    double dtf5 = 0;
    iDbg->getDebugReferencePosition(3,&dtf3);
    iDbg->getDebugReferencePosition(4,&dtf4);
    iDbg->getDebugReferencePosition(5,&dtf5);
    printf("%f \n", dtf3);
    printf("%f \n", dtf4);
    printf("%f \n", dtf5);

    //-----------------------------------------
    double posA = -0.25;
    double posB = 0.25;
    
    double starttime = Time::now();
    double endtime   = Time::now();
    double diff = endtime - starttime;
    /*while(true) {
        iDbg->setDebugReferencePosition(4,posA);
        Time::delay(0.03);
        iDbg->setDebugReferencePosition(4,posB);
        Time::delay(0.03);
        endtime   = Time::now();
        diff = endtime - starttime;
    }*/

    double pos=0;
    bool up=true;
    while(true) {
        
        iDbg->setDebugReferencePosition(4,pos);
        Time::delay(0.01);

        if (up==true) pos=pos+0.16;
        else          pos=pos-0.16;

        //0.4 & 0.1666
        if (pos>=0.4) up = false;
        if (pos<=-0.4) up =true;

        endtime   = Time::now();
        diff = endtime - starttime;
    }
  
    printf("success after all the tests \n");
    
    return 0;
}




