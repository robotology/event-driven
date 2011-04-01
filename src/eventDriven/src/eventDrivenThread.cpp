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

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/**************************************************************************/
eventDrivenThread::eventDrivenThread():currentEvent(),unmaskEvent() {
    robot = "icub";    
}

eventDrivenThread::eventDrivenThread(string _robot, string _configFile):currentEvent(),unmaskEvent()  {
    robot = _robot;
    configFile = _configFile;
    
}

eventDrivenThread::~eventDrivenThread() {
    // do nothing 
}

bool eventDrivenThread::threadInit() {
    /* open ports */ 

    EportIn.useCallback();          // to enable the port listening to events via callback
    
    if (!EportIn.open(getName(inputPortName.c_str()).c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    
    if (!eventPlot.open(getName("/AER:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
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
    
    while (isStopping() != true)  {                
        
        if(EportIn.hasNewEvent) { 
            //printf("New event received! \n");         
            currentEvent = EportIn.event; // shallow copy
            unmaskEvent.unmaskData(currentEvent.get_packet(), currentEvent.get_sizeOfPacket());
            timeBuf = unmaskEvent.getTimeBuffer(true); // why true??
            eventsBuf = unmaskEvent.getEventBuffer(true); // why true??
            plotEventBuffer(eventsBuf,128,128);
            EportIn.hasNewEvent = false;
            //printf("New event processed! \n");  
        }                      
    }
}

void eventDrivenThread::plotEventBuffer(int* buffer, int dim1, int dim2) {

    
    ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();
    unsigned char* imageTmp = imageForEventBuffer.getRawImage();
    int* bufTmp = buffer;
    for(int i = 0; i < dim1; ++i) {
        for(int j = 0; j < dim2; ++j) {
            *imageTmp = *bufTmp;
            imageTmp++;
            bufTmp++;
        }
    } 
    eventPlot.write();   

}

void eventDrivenThread::threadRelease() {
    // nothing   
}

void eventDrivenThread::onStop() {
    EportIn.interrupt();
    
    EportIn.close();
}

