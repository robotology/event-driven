// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file eventDrivenThread.cpp
 * @brief Implementation of the eventDriven thread (see eventDrivenThread.h).
 */

#include <iCub/eventDrivenThread.h>
#include <yarp/math/SVD.h>
#include <cstring>

#define LEAK_TH 1000

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

eventDrivenThread::eventDrivenThread() {
    robot = "icub";
}

eventDrivenThread::eventDrivenThread(string _robot, string _configFile) {
    robot = _robot;
    configFile = _configFile;    
}

eventDrivenThread::~eventDrivenThread() { 
}

bool eventDrivenThread::threadInit() {
    /* open ports */ 
    //EportIn.hasNewEvent = false;
    //EportIn.useCallback();          // to enable the port listening to events via callback
    leakLeft = new int[128*128];
    
    for (int j = 0; j<128*128; j++) {
        leakLeft[j]=0;
    } 

    if (!EportIn.open(getName(inputPortName.c_str()).c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!eventPlot.open(getName("/AER:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
    

}

void eventDrivenThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string eventDrivenThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void eventDrivenThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void eventDrivenThread::run() {   
   while (isStopping() != true) {
       //if(EportIn.hasNewEvent) {
           
           //printf("New event received! \n");         
           //currentEvent = EportIn.event; // shallow copy           
           currentEvent   = EportIn.read(true);    
           if(currentEvent!=NULL){
               unmaskEvent.unmaskData(currentEvent->get_packet(), currentEvent->get_sizeOfPacket());
               printf("getting buffer left after unmasking \n");
               timeBuf        = unmaskEvent.getTimeBuffer(1);      // why true??
               eventsBuf      = unmaskEvent.getEventBuffer(1);     // why true??
               printf("getting buffer right after unmasking \n");
               timeBufRight   = unmaskEvent.getTimeBuffer(0);    // why true??
               eventsBufRight = unmaskEvent.getEventBuffer(0);   // why true??
               printf("plotting events \n");
               plotEventBuffer(eventsBuf,128,128);
           }
           //EportIn.hasNewEvent = false;
           //_old printf("New event processed! \n");  
           //}
    }
}

void eventDrivenThread::plotEventBuffer(int* buffer, int dim1, int dim2) {
    ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();
    imageForEventBuffer.resize(dim1, dim2);
    //imageForEventBuffer.zero();
    unsigned char* imageTmp = imageForEventBuffer.getRawImage();
    int* bufTmp = buffer;
    for(int i = 0; i < dim1; ++i) {
        for(int j = 0; j < dim2; ++j) {
            int* leakPointer = &leakLeft[i * dim1 + j];
            if(*bufTmp != 0){
                (*leakPointer) = (*leakPointer) + 1;
                if((*leakPointer) > LEAK_TH){
                    printf("leak_th passed %d %d \n", i,j);
                    (*leakPointer) = 0;
                    printf("leak_th passed %d %d \n", i,j);
                }
                if(*bufTmp>0){
                    if(*imageTmp < 250){
                        *imageTmp = (*imageTmp)+5;
                    }
                    if(*bufTmp >= 1)
                        *bufTmp = *bufTmp - 1 ;
                }
                else {
                    if(*imageTmp < 250){
                        *imageTmp = (*imageTmp)+5;
                    }
                    if(*bufTmp <= -1)
                        *bufTmp = *bufTmp + 1 ;
                    
                }
                      
            }
            else {
                leakLeft[i * dim1 + j]--;
                if(*leakPointer < 0){
                    *leakPointer = 0;
                }
                if(*imageTmp>=1)
                    *imageTmp = *imageTmp - 1;
                
            }
            imageTmp++;
            bufTmp++;
        }
    } 
    eventPlot.write();   
}

void eventDrivenThread::threadRelease() {
    delete[] leakLeft;
    
}

void eventDrivenThread::onStop() {
    EportIn.interrupt();
    EportIn.close();
    eventPlot.interrupt();
    eventPlot.close();
}

