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
 * @file lpRemapperThread.cpp
 * @brief Implementation of the thread (see header lpRemapperThread.h)
 */

#include <iCub/lpRemapperThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

lpRemapperThread::lpRemapperThread() : RateThread(THRATE) {
    resized=false;
    count=0;
}

lpRemapperThread::~lpRemapperThread() {

}

bool lpRemapperThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    printf("starting the converter.... \n");
    lpRemapper=new logpolarRemapper();
    lpRemapper->useCallback();
    lpRemapper->open(getName("/retina:i").c_str());
    return true;
}

void lpRemapperThread::interrupt() {
    outPort.interrupt();
}

void lpRemapperThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string lpRemapperThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void lpRemapperThread::resize(int widthp, int heightp) {
}

void lpRemapperThread::run() {
    count++;
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
        if(&outputImage!=0) {
            lpRemapper->getMonoImage(&outputImage);
            outPort.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }

}

void lpRemapperThread::threadRelease() {
    outPort.close();
}

