// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
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
 * @file unmask.cpp
 * @brief A class for unmasking the event (see the header unmask.h)
 */

#include <iCub/unmask.h>
#include <cassert>

using namespace std;
using namespace yarp::os;



#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;

unmask::unmask() : RateThread(UNMASKRATETHREAD){
    numKilledEvents = 0;
    lasttimestamp = 0;
    countEvent = 0;
    countEvent2 = 0;
    minValue = 0;
    maxValue = 0;
    xmask = 0x000000fE;
    ymask = 0x00007f00;
    xmasklong = 0x000000fE;
    yshift = 8;
    yshift2= 16,
    xshift = 1;
    polshift = 0;
    polmask = 0x00000001;
    retinalSize = 128;
    temp1=true;

    buffer=new int[retinalSize*retinalSize];
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    timeBuffer=new unsigned long int[retinalSize*retinalSize];
    memset(timeBuffer,0,retinalSize*retinalSize*sizeof(unsigned int));
    fifoEvent=new int[maxPosEvent];
    memset(fifoEvent,0,maxPosEvent*sizeof(int));
    fifoEvent_temp=new int[maxPosEvent];
    memset(fifoEvent_temp,0,maxPosEvent*sizeof(int));
    fifoEvent_temp2=new int[maxPosEvent];
    memset(fifoEvent_temp2,0,maxPosEvent*sizeof(int));

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
    //uEvents = fopen("./uevents.txt","w");
}

bool unmask::threadInit() {
    return true;
}

unmask::~unmask() {
    delete[] buffer;
    delete[] timeBuffer;
    delete[] fifoEvent;
    delete[] fifoEvent_temp;
    delete[] fifoEvent_temp2;
}

void unmask::cleanEventBuffer() {
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    minValue=0;
    maxValue=0;
}

int unmask::getMinValue() {
    return minValue;
}

int unmask::getMaxValue() {
    return maxValue;
}

unsigned long int unmask::getLastTimestamp() {
    return lasttimestamp;
}

unsigned long int unmask::getEldestTimeStamp() {
    return eldesttimestamp;
}

void unmask::setLastTimestamp(unsigned long int value) {
    lasttimestamp = value;
}

int* unmask::getEventBuffer() {
    return buffer;
}

unsigned long int* unmask::getTimeBuffer() {
    return timeBuffer;
}

void unmask::run() {
    /*
    unsigned long int* pointerTime=timeBuffer;
    unsigned long int timelimit = lasttimestamp - constInterval;
    printf("last:%d \n", lasttimestamp);
    int* pointerPixel=buffer;
    for(int j=0;j<retinalSize*retinalSize;j++) {
        
        unsigned long int current = *pointerTime;
        if ((current <= timelimit)||(current >lasttimestamp)) {
            *pointerPixel == 0;
        }
        pointerTime++;
        pointerPixel++;
    }
    */
}


void unmask::unmaskData(char* i_buffer, int i_sz) {
    //cout << "Size of the received packet to unmask : " << i_sz/8<< endl;
    //AER_struct sAER

    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    
    uint32_t* buf2 = (uint32_t*)i_buffer;

    for (int evt = 0; evt < num_events; evt++) {
            // unmask the data
            unsigned long int blob = buf2[2 * evt];
            unsigned long int timestamp = buf2[2 * evt + 1];
            lasttimestamp = timestamp;
            if(evt == 0)
                eldesttimestamp = timestamp;
            
            //printf("%d \n", timestamp);

            // here we zero the higher two bytes of the address!!! Only lower 16bits used!
            blob &= 0xFFFF;
            unmaskEvent((unsigned int) blob, cartX, cartY, polarity);
            //printf(" %d : %d \n",blob,timestamp);

            if((cartX!=127)||(cartY!=0)) {      //removed one pixel which is set once the driver do not work properly
                if(polarity > 0) {
                    buffer[cartX + cartY * retinalSize] = responseGradient;
                    timeBuffer[cartX + cartY * retinalSize] = timestamp;
                    
                    if(buffer[cartX + cartY * retinalSize] > 127) {
                        buffer[cartX + cartY * retinalSize] = 127;
                    }
                }
                else if(polarity < 0) {
                    buffer[cartX + cartY * retinalSize] = -responseGradient;
                    timeBuffer[cartX + cartY * retinalSize] = timestamp;
                    
                    if (buffer[cartX + cartY * retinalSize] < -127) {
                        buffer[cartX + cartY * retinalSize] = -127;
                    }
                }
                /*
                //udpates the temporary buffer
                if(temp1) {
                    if(countEvent>maxPosEvent-1) {
                        countEvent=maxPosEvent-1;
                    }
                    fifoEvent_temp[countEvent]=cartX+cartY*retinalSize;
                    //increments the counter of events
                    countEvent++;
                }
                else {
                    if(countEvent2>maxPosEvent-1) {
                        countEvent2=maxPosEvent-1;
                    }
                    fifoEvent_temp2[countEvent2]=cartX+cartY*retinalSize;
                    //increments the counter of events
                    countEvent2++;
                }
                */
            }
            //fprintf(uEvents,"%d\t%d\t%d\t%u\n", cartX, cartY, polarity, timestamp);


    }
    //fprintf(uEvents,"%d\t%d\t%d\t%u\n", -1, -1, -1, -1);
}


void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol) {
    y = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    x = (short) ((evPU & ymask) >> yshift);
    pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
}

void unmask::unmaskEvent(long int evPU, short& x, short& y, short& pol) {
    x = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    y = (short) ((evPU & ymask) >> yshift);
    pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;        //+1 ON, -1 OFF
}

void unmask::threadRelease() {
    //no istruction in threadInit
}


/*
void unmask::unmaskData(char* i_buffer, int i_sz) {
    //cout << "Size of the received packet to unmask : " << i_sz << endl;
    //AER_struct sAER
    
    for (int j=0; j<i_sz; j+=4) {
        if((i_buffer[j+3]&0x80)==0x80) {
            // timestamp bit 15 is one -> wrap
            // now we need to increment the wrapAdd
            wrapAdd+=0x4000; //uses only 14 bit timestamps
            //System.out.println("received wrap event, index:" + eventCounter + " wrapAdd: "+ wrapAdd);
            //NumberOfWrapEvents++;
        }
        else if((i_buffer[j+3]&0x40)==0x40) {
            // timestamp bit 14 is one -> wrapAdd reset
            // this firmware version uses reset events to reset timestamps
            //write(file_desc,reset,1);//this.resetTimestamps();
            //buffer_msg[0] = 6;
            //write(file_desc,buffer_msg,1);
            wrapAdd=0;
            // log.info("got reset event, timestamp " + (0xffff&((short)aeBuffer[i]&0xff | ((short)aeBuffer[i+1]&0xff)<<8)));
        }
        else {
            //unmask the data
            unsigned int part_1 = 0x00FF&i_buffer[j];
            unsigned int part_2 = 0x00FF&i_buffer[j+1];
            unsigned int part_3 = 0x00FF&i_buffer[j+2];
            unsigned int part_4 = 0x00FF&i_buffer[j+3];
            unsigned int blob = (part_1)|(part_2<<8);
            unmaskEvent(blob, cartX, cartY, polarity);
            timestamp = ((part_3)|(part_4<<8));
            timestamp+=wrapAdd;
            if((cartX!=127)||(cartY!=0)) {      //removed one pixel which is set once the driver do not work properly
                if(polarity>0) {
                    buffer[cartX+cartY*retinalSize]=responseGradient;
                    timeBuffer[cartX+cartY*retinalSize]=timestamp;
                    lasttimestamp=timestamp;
                    if(buffer[cartX+cartY*retinalSize]>127) {
                        buffer[cartX+cartY*retinalSize]=127;
                    }
                }
                else if(polarity<0) {
                    buffer[cartX+cartY*retinalSize]=-responseGradient;
                    if (buffer[cartX+cartY*retinalSize]<-127) {
                        buffer[cartX+cartY*retinalSize]=-127;
                    }
                }
                //udpates the temporary buffer

                if(temp1) {
                    if(countEvent>maxPosEvent-1) {
                        countEvent=maxPosEvent-1;
                    }
                    fifoEvent_temp[countEvent]=cartX+cartY*retinalSize;
                    //increments the counter of events
                    countEvent++;
                }
                else {
                    if(countEvent2>maxPosEvent-1) {
                        countEvent2=maxPosEvent-1;
                    }
                    fifoEvent_temp2[countEvent2]=cartX+cartY*retinalSize;
                    //increments the counter of events
                    countEvent2++;
                }
            }
            //fprintf(uEvents,"%d\t%d\t%d\t%u\n", cartX, cartY, polarity, timestamp);
        }
    }
    //fprintf(uEvents,"%d\t%d\t%d\t%u\n", -1, -1, -1, -1);
}

*/
