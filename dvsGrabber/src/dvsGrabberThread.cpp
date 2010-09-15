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
 * @file dvsGrabberThread.cpp
 * @brief Implementation of the independent log-motion thread (see indLogMotionThread.h).
 */

#include <iCub/dvsGrabberThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;


dvsGrabberThread::dvsGrabberThread(){
    resized=false;
}

dvsGrabberThread::~dvsGrabberThread() {

}

bool dvsGrabberThread::threadInit() {
    /* open ports */

    return true;
}

void dvsGrabberThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string dvsGrabberThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void dvsGrabberThread::resize(int widthp, int heightp) {
}

void dvsGrabberThread::run() {
    while (isStopping() != true) {
        outPort.write();
    }
}

void dvsGrabberThread::onStop() {
    outPort.interrupt();
    //close ports
    outPort.close();
}

void dvsGrabberThread::threadRelease() {
}

