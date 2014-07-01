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

eventDrivenThread::eventDrivenThread():currentEvent() {
    robot = "icub";
    
    
}

eventDrivenThread::eventDrivenThread(string _robot, string _configFile):currentEvent()  {
    robot = _robot;
    configFile = _configFile;    
}

eventDrivenThread::~eventDrivenThread() {
    // do nothing
}

bool eventDrivenThread::threadInit() {

   
    EportIn.useCallback();          // to enable the port listening to events via callback

    
    if (!EportIn.open(getName(inputPortName.c_str()).c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!eventPlot.open(getName("/AER:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!EportIn.eventCoord.open(getName("/eCoord:o").c_str())) {
        cout << ": unable to open port to send event coordinates "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!EportIn.pcSz.open(getName("/packetSize:o").c_str())) {
        cout << ": unable to open port to send packet size "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    eventFrame = cvCreateImage(cvSize(128,128),IPL_DEPTH_8U, 1 ); // to remove hard coded value

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
       
          // 

            //plotEventBuffer(EportIn.bufTmp);
            EportIn.event2Frame(EportIn.bufTmp,EportIn.packet.get_sizeOfPacket(),eventFrame); 
            Time::delay(.1);
          
            
    }
}
/*
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

}*/

void eventDrivenThread::plotEventBuffer(int* buffer) {
    if(eventPlot.getOutputCount()) {
        printf("plotting \n");
        yarp::sig::ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();   
        imageForEventBuffer.resize(IMAGE_WD,IMAGE_HT);
        imageForEventBuffer.zero();
        
        unsigned char* imageT = imageForEventBuffer.getRawImage();    
        int padding =  imageForEventBuffer.getPadding();
        int* bufT = buffer;
    
        for(int i = 0; i < IMAGE_HT; ++i) {
            for(int j = 0; j < IMAGE_WD; ++j) {
                
                if(*bufT != 0){
                    if(*bufT>0){
                        if(*imageT < 250){
                            *imageT = (*imageT) + 5;
                        }
                        if(*bufT >= 1)
                            *bufT = *bufT - 1 ;
                    }
                    else {
                        if(*imageT < 250){
                            *imageT = (*imageT) + 5;
                        }
                        if(*bufT <= -1)
                            *bufT = *bufT + 1 ;                    
                    }
                    
                      
                }
                else {
                    if(*imageT>=1)
                        *imageT = *imageT - 1;
                    
                }
                imageT++;
                bufT++;                
            }
            imageT += padding;
        }        
        eventPlot.write();
    }
}


template <class eventBuffer>
void eventPort<eventBuffer>::event2Frame(int* buff, int eventSize, IplImage* retImage) {

    uchar* tmpRetImg = (uchar*)retImage->imageData;
    uchar* originImage = tmpRetImg;
    int widthRetImage = retImage->widthStep;
    unsigned int* thisEvent =  (buff);
    short int x,y,polarity,camera;
    x = y = polarity = camera = 0;
    for(int i = 0; i < eventSize; ++i) {
        unmaskOneEvent.unmaskEvent(*thisEvent,x,y,polarity,camera);
        if(polarity > .5 || polarity <-.5) {
            *(originImage + y*widthRetImage + x) = 255;
        }
        else {
            *(originImage + y*widthRetImage + x) = 0;
        }
        thisEvent++;
    }            

}

void eventDrivenThread::threadRelease() {
    // nothing
    
    
      
}

void eventDrivenThread::onStop() {
    EportIn.interrupt();
    EportIn.eventCoord.interrupt();

    EportIn.eventCoord.close();
    EportIn.close();
}

