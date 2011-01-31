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

#define MAXVALUE 4294967295
#define THRATE 10
#define STAMPINFRAME  // 10 ms of period times the us in 1 millisecond + time for computing

cfCollectorThread::cfCollectorThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    minCount = 0; //initialisation of the timestamp limits of the first frame
}

cfCollectorThread::~cfCollectorThread() {

}

bool cfCollectorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports!!!.... \n");
    outPort.open(getName("/left:o").c_str());
    outPortRight.open(getName("/right:o").c_str());
    printf("starting the converter!!!.... \n");
    cfConverter=new cFrameConverter();
    cfConverter->useCallback();
    cfConverter->open(getName("/retina:i").c_str());
    minCount = cfConverter->getEldestTimeStamp();
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
    unsigned long int l = cfConverter->getLastTimeStamp();
    if ((cfConverter->getInputCount()) && (count % 1000 == 0)) { 
        minCount = cfConverter->getEldestTimeStamp();
        
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        printf("synchronised! %d \n", minCount);
        startTimer = Time::now();
        synchronised = true;
        
    }
    

    endTimer = Time::now();
    double interval = (endTimer - startTimer) * 1000000; //interval in microsecond
    startTimer = Time::now();
    minCount += interval * 0.8;
    maxCount =  minCount + interval * 3 ;
    /*if( maxCount >= MAXVALUE) {
        printf("reached max counter \n");
        maxCount = maxCount - MAXVALUE;
    }
    */
    //maximum value  4294967295
   
    printf("interval>%f  %d,%d,%d \n",interval,minCount,l,maxCount);
    assert(maxCount < MAXVALUE);
    
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

    if(outPortRight.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImageRight=outPortRight.prepare();
        if(&outputImageRight!=0) {
            cfConverter->getMonoImage(&outputImageRight, minCount, maxCount);
            outPortRight.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }
    //minCount = cfConverter->getLastTimeStamp(); //get the last before going to sleep
    
}

void cfCollectorThread::threadRelease() {
    outPort.close();
    outPortRight.close();
}

