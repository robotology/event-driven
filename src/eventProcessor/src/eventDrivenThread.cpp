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

eventDrivenThread::eventDrivenThread():unmaskOneEvent(),currentEvent() {
    robot = "icub";
    
    
}

eventDrivenThread::eventDrivenThread(string _robot, string _configFile):unmaskOneEvent(),currentEvent()  {
    robot = _robot;
    configFile = _configFile;
}

eventDrivenThread::~eventDrivenThread() {
    // do nothing
}

bool eventDrivenThread::threadInit() {

    srand ( time(NULL) ); // To REMOVE later
    //eventCoord = new Port;
    /* open ports */ 
    //EportIn.hasNewEvent = false;
    //EportIn.useCallback();          // to enable the port listening to events via callback

    
    dumpedF.open("dumped.dat",ios::app);
    if (!EportIn.open(getName(inputPortName.c_str()).c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!eventPlot.open(getName("/AER:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!eventCoord.open(getName("/eCoord:o").c_str())) {
        cout << ": unable to open port to send event coordinates "  << endl;
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
       
            
            
            //eventBuffer* tmp = EportIn.read(false);            
            currentEvent = *EportIn.read(true);//*tmp; // deep copy
            
            if(currentEvent.get_sizeOfPacket()> 0) {

                //dumpedF.write(reinterpret_cast<char *>(currentEvent.get_packet()),currentEvent.get_sizeOfPacket());
                                
                //int randEve = rand()%(currentEvent.get_sizeOfPacket());
                unsigned int takeOneEvent; 
                //takeOneEvent = *(currentEvent.get_packet()+randEve * sizeof(unsigned int) / sizeof(char));
                printf("New event received! \n");
                short oneX,oneY,onePol, camera;
                //unmaskOneEvent.unmaskEvent(takeOneEvent,oneX,oneY,onePol,camera);
                unmaskOneEvent.unmaskData(currentEvent.get_packet(), currentEvent.get_sizeOfPacket());

                int* bufTmp = unmaskOneEvent.getEventBuffer(true);
                //Bottle& coord = eventCoord.prepare();
                                
                int maxVal = -128; int maxX = -1; int maxY = -1;
                for(int i = 0; i < 128; ++i) {
                    for(int j = 0; j < 128; ++j) {
                        if ((*bufTmp) > BUCKET_THRESHOLD && (*bufTmp) > maxVal ) {
                            maxVal = *bufTmp;
                            maxX = i;
                            maxY = j;
                        }
                        bufTmp++;
                    }
                }
        
                if(maxVal < -127) {
                    printf("No points above threshold!\n");
                }
                else {
                    //Bottle coord, rep;
                    //rep.clear(); coord.clear();
                    printf("prepare \t");
                    Bottle& coord = eventCoord.prepare();                    
                    coord.addVocab(COMMAND_VOCAB_PSAC);
                    coord.addInt(maxX);
                    coord.addInt(maxY);                    
                    //eventCoord.write(coord,rep);
                    eventCoord.write();
                    printf("finish \t");
                    //coord.clear();
                }
                maxVal = -128;
                //coord.clear();

                //timeBuf = unmaskEvent.getTimeBuffer(true); // why true??
                //eventsBuf = unmaskEvent.getEventBuffer(true); // why true??
                //plotEventBuffer(eventsBuf,128,128);
                //printf("The random %d event in the packet received was: X:%d \t Y:%d \t P:%d \t C:%d\n",randEve,oneX,oneY,onePol,camera);
                //EportIn.hasNewEvent = false;
                //printf("New event processed! \n"); 
             }
            //else printf("Something is wrong. Read a zero sized packet??\n");
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
    //delete eventCoord;
    dumpedF.close();
    
      
}

void eventDrivenThread::onStop() {
    EportIn.interrupt();
    eventCoord.interrupt();

    eventCoord.close();
    EportIn.close();
}

