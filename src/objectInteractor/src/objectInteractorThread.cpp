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

#include <iCub/objectInteractorThread.h>
#include <cstring>
#include <string>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define THRATE 10


oInteractorThread::oInteractorThread() : RateThread(THRATE) {
    count=0;
}

oInteractorThread::~oInteractorThread() {
}

bool oInteractorThread::threadInit() {
    printf("starting the thread.... \n");

    // set up the ARM MOTOR INTERFACE	
    string localName = "/" + name + "/armCtrl";
    string remoteName;

    remoteName = "/" + robotName + "/left_arm";
    //remoteName = "/" + robotName + "/right_arm";
    Property options;
    options.put("device", "remote_controlboard");	
    options.put("local", localName.c_str());                 //local port names
    options.put("remote", remoteName.c_str());				//where we connect to
    armRobotDevice = new PolyDriver(options);
    if (!armRobotDevice->isValid()) {
        printf("initialization failed: arm device not available.\n");
        return false;
    }
    armRobotDevice->view(armPos);
    armRobotDevice->view(armEnc);
    armEnc->getAxes(&jointNum);


    // SET UP the CARTESIAN INTERFACE
    localName = "/" + name + "/cartArm";
    remoteName = "/" + robotName + "/cartesianController/left_arm";
    //remoteName = "/cartesianSolver/left_arm";
    //remoteName = "/" + robotName + "/cartesianController/right_arm";



    options.clear();
    options.put("device","cartesiancontrollerclient");
    options.put("local", localName.c_str());                //local port names
    options.put("remote", remoteName.c_str());              //where we connect to	 
    cartCtrlDevice = new PolyDriver(options);
    if (!cartCtrlDevice->isValid()) {
       printf("initialization failed: cartesian controller not available.\n");
       return false;
    }
    cartCtrlDevice->view(armCart);

    /* open ports */
    printf("opening ports.... \n");
    locationPort.open(getName("/location:i").c_str());
    return true;
}

void oInteractorThread::interrupt() {
    locationPort.interrupt();
}

void oInteractorThread::setRobotName(std::string str) {
    this->robotName=str;
    printf("name: %s", name.c_str());
}

void oInteractorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string oInteractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void oInteractorThread::run() {
    count++;
    if(locationPort.getInputCount()) {
        Bottle* locBottle = &locationPort.prepare();
        locBottle=locationPort.read();
        if(locBottle!=0) {
            printf("Bottle: %s \n", locBottle->toString().c_str());
            Bottle* list = locBottle->get(1).asList();
            if (list!=0) {
                double x = list->get(0).asDouble();
                double y = list->get(1).asDouble();
                double z = list->get(2).asDouble();
                printf("x:%f,y:%f,z:%f \n",x,y,z);
                if((x>=-0.4) &&(x<=-0.2)&&(y>=-0.4)&&(y<=0.2)&&(z>=-0.1)&&(z<=0.5)) {
                    Vector xd(3);
                    xd(0) = x;
                    xd(1) = y;
                    xd(2) = z;
                    armCart->goToPositionSync(xd);
                }
            }
            int command = locBottle->get(0).asVocab();
        }
    }
}


void oInteractorThread::threadRelease() {
    //closing ports
    locationPort.close();
    //closing kinematic interfaces
    armCart->stopControl();
    armPos->stop();
    armRobotDevice->close();
    cartCtrlDevice->close();
    delete armPos;
    delete armCart;
    delete armRobotDevice;
    delete cartCtrlDevice;
}

