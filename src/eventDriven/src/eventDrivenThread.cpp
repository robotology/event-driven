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

#define LEAK_TH 300
#define BUCKET_THRESHOLD 112

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

eventDrivenThread::eventDrivenThread() {
    robot = "icub";
    zerod = false;
}

eventDrivenThread::eventDrivenThread(string _robot, string _configFile) {
    robot = _robot;
    configFile = _configFile;    
    zerod = false;
}

eventDrivenThread::~eventDrivenThread() { 
}

bool eventDrivenThread::threadInit() {
    /* open ports */ 
    EportIn.hasNewEvent = false;
    EportIn.useCallback();          // to enable the port listening to events via callback
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
    if (!commandOut.open(getName("/command:o").c_str())) {
        cout <<": unable to open port for sending commands  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!pcSz.open(getName("/packetSize:o").c_str())) {
        cout <<": unable to open port for sending commands  "  << endl;
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
       if(EportIn.hasNewEvent) {
           EportIn.setHasNewEvent(false);
           currentEventTrain = &EportIn.eventTrain; // shallow copy           
           //currentEvent   = EportIn.read(true);    
           int dim = currentEventTrain->get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
                     
           if(currentEventTrain->get_sizeOfPacket() != 0) {
         
               unmaskEvent.unmaskData(currentEventTrain->get_packet(), currentEventTrain->get_sizeOfPacket());
               
               //timeBuf        = unmaskEvent.getTimeBuffer(1);      
               eventsBuf      = unmaskEvent.getEventBuffer(1);     
               //timeBufRight   = unmaskEvent.getTimeBuffer(0);   
               //eventsBufRight = unmaskEvent.getEventBuffer(0);                 
               plotEventBuffer(eventsBuf,128,128);
              
               /*
               if(pcSz.getOutputCount()) {
                   int packetSize = currentEvent->get_sizeOfPacket();
                   yarp::os::Bottle& b = pcSz.prepare();
                   b.clear();
                   b.addInt(packetSize);
                   pcSz.write();
               }
               */
               
               
               //unmaskEvent.unmaskData(packet.get_packet(), packetSize);
               //int* bufTmp = unmaskEvent.getEventBuffer(true);        // true for left camera
               //int* bufTmpRight = unmaskEvent.getEventBuffer(false);  // false for right camera
               
            
               
               
               /*
                 for(int i = 0; i < IMAGE_HT; ++i) {
                 for(int j = 0; j < IMAGE_WD; ++j) {
                 if ((*bufTmp) > BUCKET_THRESHOLD && (*bufTmp) > maxVal ) {
                 maxVal = *bufTmp;
                 maxX = i;
                 maxY = j;
                 }
                 if ((*bufTmpRight) > BUCKET_THRESHOLD && (*bufTmpRight) > maxVal ) {
                 maxValR = *bufTmpRight;
                 maxXR = i;
                 maxYR = j;
                 }
                 bufTmpRight++;
                 bufTmp++;
                 }
                 }
                 
                 if(maxVal < -IMAGE_WD) {
                 printf("No points above threshold!\n");
                 }
                 else {
                 
                 yarp::os::Bottle& coord = commandOut.prepare();
                 coord.clear();                    
                 coord.addInt(maxVal);
                 coord.addInt(maxX);
                 coord.addInt(maxY);
                 coord.addString(" R:");
                 coord.addInt(maxValR);
                 coord.addInt(maxXR);
                 coord.addInt(maxYR);                    
                 commandOut.write();
                 
                 
                 }
                 maxVal = -IMAGE_WD -1;        
                 packetSize = 0;
               */
           }
       }                                  
    }
}

void eventDrivenThread::plotEventBuffer(int* buffer, int dim1, int dim2) {
    if(eventPlot.getOutputCount()) {
        
        ImageOf<yarp::sig::PixelMono>& imageForEventBuffer = eventPlot.prepare();   
        imageForEventBuffer.resize(dim1, dim2);
        if(!zerod) {
            imageForEventBuffer.zero();
            zerod = true;
        }
        
        unsigned char* imageTmp = imageForEventBuffer.getRawImage();    
        int padding =  imageForEventBuffer.getPadding();
        int* bufTmp = buffer;
    
        for(int i = 0; i < dim1; ++i) {
            for(int j = 0; j < dim2; ++j) {
                
                //int* leakPointer = &leakLeft[i * dim1 + j];
                if(*bufTmp != 0){
                    //(*leakPointer) = (*leakPointer) + 1;
                    
                    /*
                    if((*leakPointer) > LEAK_TH){
                        //commandOut.clear();
                        if(commandOut.getOutputCount()) {
                            Bottle& b = commandOut.prepare();
                            b.clear();
                            b.addString("SAC_MONO");
                            b.addInt(i);
                            b.addInt(j);
                            b.addDouble(0.5);
                            printf("leak_th passed %d %d \n", i,j);
                            commandOut.write();
                        }
                        
                        //(*leakPointer) = 0;
                        printf("leak_th passed %d %d \n", i,j);
                    }
                    */
                                        
                    if(*bufTmp>0){
                        if(*imageTmp < 254){
                            *imageTmp = (*imageTmp) + 1;
                        }
                        else {
                            //resetting the response
                            *imageTmp = 0;
                            //sending out the location
                            if(commandOut.getOutputCount()) {
                                Bottle& b = commandOut.prepare();
                                b.clear();
                                b.addString("SAC_MONO");
                                b.addInt(i);
                                b.addInt(j);
                                b.addDouble(0.5);
                                
                                commandOut.write();
                            }
                        }
                        if(*bufTmp >= 1)
                            *bufTmp = *bufTmp - 1 ;
                    }
                    else {
                        if(*imageTmp < 254){
                            *imageTmp = (*imageTmp) + 1;
                        }
                        else {
                            //resetting the response
                            *imageTmp = 0;
                            //sending out the location
                            if(commandOut.getOutputCount()) {
                                Bottle& b = commandOut.prepare();
                                b.clear();
                                b.addString("SAC_MONO");
                                b.addInt(i);
                                b.addInt(j);
                                b.addDouble(0.5);   
                                commandOut.write();
                            }
                        }
                        if(*bufTmp <= -1)
                            *bufTmp = *bufTmp + 1 ;                    
                    }
                    
                      
                }
                else {
                    //leakLeft[i * dim1 + j]--;
                    //if(*leakPointer < 0){
                    //    *leakPointer = 0;
                    //}
                    if(*imageTmp>=5)
                        *imageTmp = *imageTmp - 5;
                    
                }
                imageTmp++;
                bufTmp++;                
            }
            imageTmp += padding;
        }        
        eventPlot.write();
    }
}

void eventDrivenThread::threadRelease() {
    //delete[] leakLeft;    
}

void eventDrivenThread::onStop() {
    EportIn.interrupt();
    EportIn.close();
    eventPlot.interrupt();
    eventPlot.close();
    commandOut.interrupt();
    commandOut.close();
    printf("closed all the ports \n");
}

