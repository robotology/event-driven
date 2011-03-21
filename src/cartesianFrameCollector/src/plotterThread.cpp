
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file plotterThread.cpp
 * @brief Implementation of the thread that represent the frame conversion of events (see header plotterThread.h)
 */

#include <iCub/plotterThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 30 
#define retinalSize 128

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    imageLeft = 0;
    imageRight = 0;
}

plotterThread::~plotterThread() {
    printf("freeing memory in collector");
    
}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    /* open ports */
    leftPort.open(getName("/left:o").c_str());
    rightPort.open(getName("/right:o").c_str());    
    return true;
}

void plotterThread::interrupt() {
    leftPort.interrupt();
    rightPort.interrupt();
}

void plotterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string plotterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void plotterThread::resize(int widthp, int heightp) {
}

void plotterThread::copyLeft(ImageOf<PixelMono>* image) {
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0; c < retinalSize; c++) {
                *pleft++ = *pimage++;
            }
            pleft += padding;
            pimage += padding;
        }
    }
}

void plotterThread::copyRight(ImageOf<PixelMono>* image) {
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pright = imageRight->getRawImage();
    if(imageRight != 0) {
        for(int r = 0;r < retinalSize; r++) {
            for(int c = 0;c < retinalSize; c++) {
                *pright++ = *pimage++;
            }
            pright += padding;
            pimage += padding;
        }
    }
}


void plotterThread::run() {

    imageLeft  = &leftPort.prepare();
    imageLeft->resize(retinalSize, retinalSize);
    imageRight = &rightPort.prepare();
    imageRight->resize(retinalSize, retinalSize);
    synchronised = true;

    if(leftPort.getOutputCount()) {
        leftPort.write();
    }
    if(rightPort.getOutputCount()) {
        rightPort.write();
    }
}


void plotterThread::threadRelease() {
    leftPort.close();
    rightPort.close();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
