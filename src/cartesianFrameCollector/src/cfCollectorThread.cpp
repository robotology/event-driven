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
#include <time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define MAXVALUE 4294967295
#define THRATE 30
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
    clock(); //startTime ;
    //T1 = times(&start_time);
    microseconds = 0;
    microsecondsPrev = 0;
    gettimeofday(&tvstart, NULL);
    
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
    if(count == 100000) {
        count = 0;
    }
    if(!synchronised) {
        countDivider = count; 
    }
    //T2 = times(&stop_time);
    unsigned long int l = cfConverter->getLastTimeStamp();
    gettimeofday(&tvend, NULL);
    Tnow = ((u64)tvend.tv_sec) * 1000000 + ((u64)tvstart.tv_usec);
    
    endTimer = Time::now();
    clock_gettime(CLOCK_REALTIME, &stop_time );
    double diffTime = (endTime - startTime);
    printf("timeofday>%ld\n", ((tvend.tv_sec * 1000000 + tvend.tv_usec)
		  - (tvstart.tv_sec * 1000000 + tvstart.tv_usec)));
    double interval = (endTimer - startTimer) * 1000000; //interval in microsecond
    //double time = (double)stop_time.tms_utime - start_time.tms_utime;
    microseconds = stop_time.tv_nsec / 1000 ; 
    startTimer = Time::now();
    gettimeofday(&tvstart, NULL);
    //startTime = clock();
    //T1 = times(&start_time);
    //clock();
    //clock_gettime( CLOCK_REALTIME, &start_time );
    
    if ((cfConverter->getInputCount()) && (count % 500 == 0)) { 
        minCount = l - interval; //cfConverter->getEldestTimeStamp();        
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
    
    
    minCount += interval * 0.9;
    maxCount =  minCount + interval ;
    //if (l > maxCount)
    //    printf("Error \n");
    
    /*if( maxCount >= MAXVALUE) {
        printf("reached max counter \n");
        maxCount = maxCount - MAXVALUE;
    }
    */
    //maximum value  4294967295
   
    printf("interval>%f diffTime>%f  %d,%d,%d \n",interval,microseconds - microsecondsPrev,minCount,l,maxCount);
    microsecondsPrev = microseconds;
    if(outPort.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
        if(&outputImage!=0) {
            cfConverter->getMonoImage(&outputImage, minCount, maxCount,1);
            outPort.write();
        }
        else {
            printf("reference to the outimage null \n");
        }
    }

    if(outPortRight.getOutputCount()) {
        ImageOf<yarp::sig::PixelMono>& outputImageRight=outPortRight.prepare();
        if(&outputImageRight!=0) {
            printf("asking for right image \n");
            cfConverter->getMonoImage(&outputImageRight, minCount, maxCount, 0);
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

