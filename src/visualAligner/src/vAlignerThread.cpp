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
 * @file vAlignerThread.cpp
 * @brief Implementation of the thread (see header vaThread.h)
 */

#include <iCub/vAlignerThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

vAlignerThread::vAlignerThread() : RateThread(THRATE) {
    resized=false;
    count=0;
}

vAlignerThread::~vAlignerThread() {

}

bool vAlignerThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    outPort.open(getName("/image:o").c_str());
    printf("starting the converter.... \n");
    return true;
}

void vAlignerThread::interrupt() {
    outPort.interrupt();
}

void vAlignerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string vAlignerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void vAlignerThread::resize(int widthp, int heightp) {
}

void vAlignerThread::run() {
    count++;
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
    }

}

void vAlignerThread::threadRelease() {
    outPort.close();
}

