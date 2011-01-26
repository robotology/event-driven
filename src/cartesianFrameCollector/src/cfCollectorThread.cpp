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
 * @file cfCollectorThread.cpp
 * @brief Implementation of the thread (see header cfCollectorThread.h)
 */

#include <iCub/cfCollectorThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing

cfCollectorThread::cfCollectorThread() : RateThread(THRATE) {
    resized=false;
    count=0;
    minCount = 0; //initialisation of the timestamp limits of the first frame
}

cfCollectorThread::~cfCollectorThread() {

}

bool cfCollectorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports!!!.... \n");
    outPort.open(getName("/image:o").c_str());
    printf("starting the converter!!!.... \n");
    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->open(getName("/retina:i").c_str());
    maxCount = cfConverter->getLastTimeStamp();
    startTimer = Time::now();
    return true;
}

void cfCollectorThread::interrupt() {
    outPort.interrupt();
}

void cfCollectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string cfCollectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void cfCollectorThread::resize(int widthp, int heightp) {
}

void cfCollectorThread::run() {
    count++;
    if (count % 1 == 0) {
         maxCount = cfConverter->getLastTimeStamp();
         //startTimer = Time::now();
    }
    endTimer = Time::now();
    double interval = (endTimer - startTimer) * 1000000; //interval in microsecond
    startTimer = Time::now();
    maxCount += interval;
    minCount =  maxCount - interval * 1.5;
    if( maxCount >= 2147483647) {
        maxCount = maxCount - 2147483647;
    }
    //maximum value  2147483647 4294967295
    long int l = cfConverter->getLastTimeStamp();
    printf("interval:%f  %d,%d,%d \n",maxCount,interval,minCount,l);
    assert(maxCount < 2147483647);
    
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
        if(&outputImage!=0) {
            cfConverter->getMonoImage(&outputImage, minCount, maxCount);
            outPort.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }

    minCount = cfConverter->getLastTimeStamp(); //get the last before going to sleep
    
}

void cfCollectorThread::threadRelease() {
    outPort.close();
}

