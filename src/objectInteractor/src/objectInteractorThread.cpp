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
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10


oInteractorThread::oInteractorThread() : RateThread(THRATE) {

    count=0;

}

oInteractorThread::~oInteractorThread() {
}

bool oInteractorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    //outPort.open(getName("/image:o").c_str());
    return true;
}

void oInteractorThread::interrupt() {

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

}


void oInteractorThread::threadRelease() {

}

