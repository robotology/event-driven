// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco,Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file disparitySelectorThread.cpp
 * @brief Implementation of the disparitySelector thread (see disparitySelectorThread.h).
 */

#include <iCub/disparitySelectorThread.h>
#include <yarp/math/SVD.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

disparitySelectorThread::disparitySelectorThread() {
    robot = "icub";
}

disparitySelectorThread::disparitySelectorThread(string _robot, string _configFile) {
    robot = _robot;
    configFile = _configFile;    
}

disparitySelectorThread::~disparitySelectorThread() {
    // do nothing 
}

bool disparitySelectorThread::threadInit() {
    /* open ports */ 
    //EportIn.hasNewEvent = false;
    //EportIn.useCallback();          // to enable the port listening to events via callback
    
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

void disparitySelectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string disparitySelectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void disparitySelectorThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void disparitySelectorThread::run() {   
   while (isStopping() != true) {
       //if(EportIn.hasNewEvent) {
           
           //printf("New event received! \n");         
           //currentEvent = EportIn.event; // shallow copy           
           currentEvent   = EportIn.read(true);    
           unmaskEvent.unmaskData(currentEvent->get_packet(), currentEvent->get_sizeOfPacket());
           timeBuf        = unmaskEvent.getTimeBuffer(1);      // why true??
           eventsBuf      = unmaskEvent.getEventBuffer(1);     // why true??
           timeBufRight   = unmaskEvent.getTimeBuffer(0);    // why true??
           eventsBufRight = unmaskEvent.getEventBuffer(0);   // why true??
           plotEventBuffer(eventsBuf,128,128);
           //EportIn.hasNewEvent = false;
            //_old printf("New event processed! \n");  
           //}
       
    }
}

void disparitySelectorThread::plotEventBuffer(int* buffer, int dim1, int dim2) {
    ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();
    imageForEventBuffer.resize(dim1, dim2);
    //imageForEventBuffer.zero();
    unsigned char* imageTmp = imageForEventBuffer.getRawImage();
    int* bufTmp = buffer;
    for(int i = 0; i < dim1; ++i) {
        for(int j = 0; j < dim2; ++j) {
            
            if(*bufTmp != 0){
                if(*bufTmp>0){
                    //printf("%d \n", *bufTmp);
                    if(*imageTmp < 250){
                        *imageTmp = (*imageTmp)+5;
                        //*imageTmp = 255;
                    }
                    if(*bufTmp >= 1)
                        *bufTmp = *bufTmp - 1 ;
                }
                else {
                    if(*imageTmp < 250){
                        *imageTmp = (*imageTmp)+5;
                        //*imageTmp = 255;
                    }
                    if(*bufTmp <= -1)
                        *bufTmp = *bufTmp + 1 ;
                    
                }
                      
            }
            else {
                if(*imageTmp>=1)
                    *imageTmp = *imageTmp - 1;
                
            }
            imageTmp++;
            bufTmp++;
        }
    } 
    eventPlot.write();   
}

void disparitySelectorThread::threadRelease() {
    // nothing   
}

void disparitySelectorThread::onStop() {
    EportIn.interrupt();
    EportIn.close();
    eventPlot.interrupt();
    eventPlot.close();
}

