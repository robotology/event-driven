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
 * Public License for more details
 */

/**
 * @file logUnmask.cpp
 * @brief A class for unmasking the logEvent (see the header unmask.h)
 */

#include <iCub/logUnmask.h>
#include <math.h>
#include <cassert>

using namespace std;
using namespace yarp::os;

//#define LINUX
//#ifndef LINUX
// #ifndef __linux__ || linux
#ifndef __linux__              // posix compliant and for GCC
typedef unsigned long uint32_t;
#endif 

#define MAXVALUE 114748364 //4294967295
#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;


logUnmask::logUnmask() : RateThread(UNMASKRATETHREAD){
    count = 0;
    verb = false;
    numKilledEvents = 0;
    lasttimestamp = 0;
    validLeft = false;
    validRight = false;
    eldesttimestamp = MAXVALUE;
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
    camerashift = 15;
    cameramask = 0x00008000;
    retinalSize = 128;
    temp1=true;

    buffer=new int[retinalSize*retinalSize];
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    timeBuffer=new unsigned long[retinalSize*retinalSize];
    memset(timeBuffer,0,retinalSize*retinalSize*sizeof(unsigned long));
    bufferRight=new int[retinalSize*retinalSize];
    memset(bufferRight,0,retinalSize*retinalSize*sizeof(int));
    timeBufferRight=new unsigned long[retinalSize*retinalSize];
    memset(timeBufferRight,0,retinalSize*retinalSize*sizeof(unsigned long));
    
    /*fifoEvent=new int[maxPosEvent];
    memset(fifoEvent,0,maxPosEvent*sizeof(int));
    fifoEvent_temp=new int[maxPosEvent];
    memset(fifoEvent_temp,0,maxPosEvent*sizeof(int));
    fifoEvent_temp2=new int[maxPosEvent];
    memset(fifoEvent_temp2,0,maxPosEvent*sizeof(int));
    */

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the logUnmasked_buffer
    //uEvents = fopen("./uevents.txt","w");

    logChip_LUT = new feature[648]; // all the position in the logchip times 4 features 
}

bool logUnmask::threadInit() {
    // initialising the logChip_LUT

    
    /*  // structure of the LUT for logpolar Chip
    logChip_LUT[0  ][0] = 1 ; logChip_LUT[0  ][1] = 0 ; logChip_LUT[0  ][2] = 0 ; logChip_LUT[0 ][3] = 1;
    logChip_LUT[1  ][0] = 1 ; logChip_LUT[1  ][1] = 0 ; logChip_LUT[1  ][2] = 0 ; logChip_LUT[1 ][3] = 0;
    logChip_LUT[6  ][0] = 5 ; logChip_LUT[6  ][1] = 0 ; logChip_LUT[6  ][2] = 0 ; logChip_LUT[6 ][3] = 1;
    logChip_LUT[10 ][0] = 2 ; logChip_LUT[10 ][1] = 0 ; logChip_LUT[10 ][2] = 0 ; logChip_LUT[10][3] = 1;
    logChip_LUT[11 ][0] = 2 ; logChip_LUT[11 ][1] = 0 ; logChip_LUT[11 ][2] = 0 ; logChip_LUT[11][3] = 0;
    logChip_LUT[12 ][0] = 1 ; logChip_LUT[12 ][1] = 0 ; logChip_LUT[12 ][2] = 2 ; logChip_LUT[12][3] = 1;
    logChip_LUT[13 ][0] = 1 ; logChip_LUT[13 ][1] = 0 ; logChip_LUT[13 ][2] = 2 ; logChip_LUT[13][3] = 0;
    logChip_LUT[18 ][0] = 5 ; logChip_LUT[18 ][1] = 0 ; logChip_LUT[18 ][2] = 2 ; logChip_LUT[18][3] = 1;
    logChip_LUT[22 ][0] = 2 ; logChip_LUT[22 ][1] = 0 ; logChip_LUT[22 ][2] = 2 ; logChip_LUT[22][3] = 1;
    logChip_LUT[23 ][0] = 2 ; logChip_LUT[23 ][1] = 0 ; logChip_LUT[23 ][2] = 2 ; logChip_LUT[23][3] = 0;
    logChip_LUT[24 ][0] = 1 ; logChip_LUT[24 ][1] = 0 ; logChip_LUT[24 ][2] = 4 ; logChip_LUT[24][3] = 1;
    logChip_LUT[25 ][0] = 1 ; logChip_LUT[25 ][1] = 0 ; logChip_LUT[25 ][2] = 4 ; logChip_LUT[25][3] = 0;
    logChip_LUT[30 ][0] = 5 ; logChip_LUT[30 ][1] = 0 ; logChip_LUT[30 ][2] = 4 ; logChip_LUT[30][3] = 1;
    logChip_LUT[34 ][0] = 2 ; logChip_LUT[34 ][1] = 0 ; logChip_LUT[34 ][2] = 4 ; logChip_LUT[34][3] = 1;
    logChip_LUT[35 ][0] = 2 ; logChip_LUT[35 ][1] = 0 ; logChip_LUT[35 ][2] = 4 ; logChip_LUT[35][3] = 0;

    logChip_LUT[38 ][0] = 0 ; logChip_LUT[38 ][1] = 0 ; logChip_LUT[38 ][2] = 0 ; logChip_LUT[38 ][3] = 1;
    logChip_LUT[39 ][0] = 0 ; logChip_LUT[39 ][1] = 0 ; logChip_LUT[39 ][2] = 0 ; logChip_LUT[39 ][3] = 0;
    logChip_LUT[50 ][0] = 0 ; logChip_LUT[50 ][1] = 0 ; logChip_LUT[50 ][2] = 2 ; logChip_LUT[50 ][3] = 1;
    logChip_LUT[51 ][0] = 0 ; logChip_LUT[51 ][1] = 0 ; logChip_LUT[51 ][2] = 2 ; logChip_LUT[51 ][3] = 0;
    logChip_LUT[62 ][0] = 0 ; logChip_LUT[62 ][1] = 0 ; logChip_LUT[62 ][2] = 4 ; logChip_LUT[62 ][3] = 1;
    logChip_LUT[63 ][0] = 0 ; logChip_LUT[63 ][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0; 

    logChip_LUT[216][0] = 0 ; logChip_LUT[216][1] = 0 ; logChip_LUT[216][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[217][0] = 0 ; logChip_LUT[217][1] = 0 ; logChip_LUT[217][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[222][0] = 0 ; logChip_LUT[222][1] = 0 ; logChip_LUT[222][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[226][0] = 0 ; logChip_LUT[226][1] = 0 ; logChip_LUT[226][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[227][0] = 0 ; logChip_LUT[227][1] = 0 ; logChip_LUT[227][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[228][0] = 0 ; logChip_LUT[228][1] = 0 ; logChip_LUT[228][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[229][0] = 0 ; logChip_LUT[229][1] = 0 ; logChip_LUT[229][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[232][0] = 0 ; logChip_LUT[232][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[233][0] = 0 ; logChip_LUT[233][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[234][0] = 0 ; logChip_LUT[234][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[235][0] = 0 ; logChip_LUT[235][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[238][0] = 0 ; logChip_LUT[238][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[239][0] = 0 ; logChip_LUT[239][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[240][0] = 0 ; logChip_LUT[240][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[241][0] = 0 ; logChip_LUT[241][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[246][0] = 0 ; logChip_LUT[246][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[250][0] = 0 ; logChip_LUT[250][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    logChip_LUT[251][0] = 0 ; logChip_LUT[251][1] = 0 ; logChip_LUT[63 ][2] = 4 ; logChip_LUT[63 ][3] = 0;
    */

    int type, metax, metay, pol;  // feature listed in the same order as in the LUT

    for (int x = 0; x < 36; x++ ) {
        for (int y = 0; y < 18; y++) {
            
            if ((x-2) % 6 == 0){
                // CD
                type  = 0;
                metax = (int) (x - 2) / 6;
                metay = (int) (y - 1) / 3;
                if(x == 20) {
                    pol = 0;
                }
                else if(x == 21) {
                    pol = 1;
                }
                else if(x % 2 == 0) {
                    pol = 1;
                }
                else {
                    pol = 0; 
                }
            } // CD            
            
            if (((x - 6) % 12 == 0) && ((y >= 12) || (y <= 5))) {
                // IF
                type  = 5;
                metax = (int) (x - 6) / 6;
                metay = (y % 2 == 0) ? y / 3 : (y / 3) - 1;
                if(x % 2 == 0) {
                    pol = 1;
                }
                else {
                    pol = 0; 
                }
            } // IF
            
            
            // EM 
            if((y >= 6) && (y < 12) && (x >= 12) && (x < 24)) {
                //fovea
                if((x >= 16) && (x < 20)) {
                    //inner columns
                    if (y % 3 == 0) {
                        type = 2;
                        metax = (x < 18)     ? 2 : 3;
                        metay = (y % 2 == 0) ? 2 : 3;
                        if((x == 16) || (x == 19)) {
                            pol = 1;
                        }
                        else {
                            pol = 0;
                        }
                    }                      
                    else {
                        type = 4;
                        metax = (x < 18)     ? 2 : 3;
                        metay = (y % 2 == 0) ? 2 : 3;
                        if((x == 16)||(x == 19)) {
                            pol = 1;
                        }
                        else {
                            pol = 0;
                        }
                    }                    
                }  // inner columns
                else {
                    //outer columns
                    if (y % 3 == 0) { 
                        type = 1;
                        metax = (x < 18)     ? 2 : 3;                        
                        metay = (y % 2 == 0) ? 2 : 3;                      
                        if((x == 12)||(x == 23)) {
                            pol = 1;
                        }
                        else {
                            pol = 0;
                        }
                    }
                    else { 
                        type = 3 ;
                        metax = (x < 18)     ? 2 : 3;
                        metay = (y % 2 == 0) ? 2 : 3;
                        if((x == 12)||(x == 23)) {
                            pol = 1;
                        }
                        else {
                            pol = 0;
                        }
                    }
                } // outer columns                           
            }// fovea                
            else {
                //periphery
                if ( ( x       % 6 == 0 ) && (  y % 3      == 0) ) {
                    type = 1;
                    metax =  x / 6;
                    metay =  y / 3;
                    if (x % 2 == 0) {
                        pol = 1;
                    }
                    else {
                        pol = 0;
                    }
                }
                if ( ((x - 10) % 6 == 0 ) && (  y % 3      == 0) ) {
                    type = 2 ;
                    metax = (x - 10) / 6;
                    metay =  y / 3;
                    if (x % 2 == 0) {
                        pol = 1;
                    }
                    else {
                        pol = 0;
                    }
                }
                if ( ( x       % 6 == 0 ) && ( (y - 5) % 3 == 0) ) { 
                    type= 3;
                    metax =  x / 6;
                    metay = (y - 5)  / 3;
                    if (x % 2 == 0) {
                        pol = 1;
                    }
                    else {
                        pol = 0;
                    }
                }
                if ( ((x - 10) % 6 == 0 ) && ( (y - 5) % 3 == 0) ) {
                    type= 4;
                    metax = (x - 10) / 6;
                    metay = (y - 5)  / 3;
                    if (x % 2 == 0) {
                        pol = 1;
                    }
                    else {
                        pol = 0;
                    }
                }
            } // periphery             
            logChip_LUT[y * 36 + x][0] = type ; logChip_LUT[y * 36 + x][1] = metax ; logChip_LUT[y * 36 + x][2] = metay ; logChip_LUT[y * 36 + x][3] = pol;
        }
    }
    return true;
}

logUnmask::~logUnmask() {
    delete[] buffer;
    delete[] timeBuffer;
    delete[] bufferRight;
    delete[] timeBufferRight;
    delete[] logChip_LUT;
}

void logUnmask::cleanEventBuffer() {
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    memset(timeBuffer, 0, retinalSize * retinalSize * sizeof(unsigned long));
    minValue=0;
    maxValue=0;
}

int logUnmask::getMinValue() {
    return minValue;
}

int logUnmask::getMaxValue() {
    return maxValue;
}

unsigned long logUnmask::getLastTimestamp() {
    return lasttimestamp;
}

unsigned long logUnmask::getLastTimestampRight() {
    return lasttimestampright;
}

unsigned long logUnmask::getEldestTimeStamp() {
    return eldesttimestamp;
}

void logUnmask::setLastTimestamp(unsigned long value) {
    lasttimestamp = value;
}

int* logUnmask::getEventBuffer(bool camera) {
    if(camera)
        return buffer;
    else
        return bufferRight;
}

void logUnmask::resetTimestamps() {
    for (int i=0 ; i<retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
        timeBufferRight[i] = 0;
    }
    //verb = true;
    //lasttimestamp = 0;
    //lasttimestampright = 0;  
}

unsigned long* logUnmask::getTimeBuffer(bool camera) {
    if (camera)
        return timeBuffer;
    else
        return timeBufferRight;
}

void logUnmask::run() {
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


void logUnmask::logUnmaskData(char* i_buffer, int i_sz, bool verb) {
    //cout << "Size of the received packet to logUnmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    //AER_struct sAER
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //create a pointer that points every 4 bytes
    uint32_t* buf2 = (uint32_t*)i_buffer;
    //eldesttimestamp = 0;
    for (int evt = 0; evt < num_events; evt++) {
        
        // logUnmask the data ( first 4 byte blob, second 4 bytes timestamp)
        unsigned long blob      = buf2[2 * evt];
        unsigned long timestamp = buf2[2 * evt + 1];
        //printf("0x%x 0x%x \n",blob, timestamp);
        
        // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        blob &= 0xFFFF;
        logUnmaskEvent((unsigned int) blob, cartX, cartY, polarity, type);
        //if(count % 100 == 0) {
        //    printf(" %d>%d,%d : %d : %d \n",blob,cartX,cartY,timestamp,camera);
        //}
        cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        cartX = retinalSize - cartX;
        
        switch (type) {
        case 0: //CD
            break;
        case 1: //EM1
            break;
        case 2: //EM2
            break;
        case 3: //EM3
            break;
        case 4: //EM4
            break;
        case 5: //IF
            break;
        }
        
        /*
        //camera: LEFT 0, RIGHT 1
        if(camera) {            
            if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
                validLeft =  true;
            }
            
            if(verb) {
                lasttimestamp = 0;
                resetTimestamps();
            }
            else if(timestamp > lasttimestamp) {
                lasttimestamp = timestamp;
            }
            
            
            
            if(timeBuffer[cartX + cartY * retinalSize] < timestamp) {
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
            }
           
        }
        else {
            if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
                validRight =  true;
            }

            if(verb) {
               lasttimestampright = 0;
               resetTimestamps();
            }
            else if( timestamp > lasttimestampright){
                lasttimestampright = timestamp;
            }
           

            if (timeBufferRight[cartX + cartY * retinalSize] < timestamp) {
                if(polarity > 0) {
                    bufferRight[cartX + cartY * retinalSize] = responseGradient;
                    timeBufferRight[cartX + cartY * retinalSize] = timestamp;
                    
                    if(bufferRight[cartX + cartY * retinalSize] > 127) {
                        bufferRight[cartX + cartY * retinalSize] = 127;
                    }
                }
                else if(polarity < 0) {
                    bufferRight[cartX + cartY * retinalSize] = -responseGradient;
                    timeBufferRight[cartX + cartY * retinalSize] = timestamp;
                    
                    if (bufferRight[cartX + cartY * retinalSize] < -127) {
                        bufferRight[cartX + cartY * retinalSize] = -127;
                    }
                }
            }
        }
        */
    }
}




void logUnmask::logUnmaskEvent(unsigned int evPU, short& x, short& y, short& pol, short& type) {
    y = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    x = (short) ((evPU & ymask) >> yshift);
    pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    type = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
}



/*

void logUnmask::logUnmaskEvent(long int evPU, short& metax, short& metay, short& pol, short& type) {
    int x    = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    int y    = (short) ((evPU & ymask) >> yshift);
    pol  = ((short)((evPU & polmask) >> polshift)==0)?-1:1;        //+1 ON, -1 OFF
    //determining whether the address is in fovea or in periphery

    if ((x-2)%6==0){
        // CD
        type = 0;
        x = (int) (x - 2) / 6;
        y = (int) (y - 1) / 3;
    }
    
    if ((x-6)%12==0)&&((y>=12)||(y<=5)) {
        // IF
        type =  5;
        x = (int) (x - 6) / 6;
        if (y % 2 == 0)
            y = (int)  y / 3;
        else
            y = (int)  (y / 3) - 1; 
    }

    if((y>=6)&&(y<12)&&(x>=12)&&(x<24)) {
        //fovea
        if((x >= 16) && (x < 20)) {
            //inner columns
            if (y % 3 == 0) {
                type = 2;
                if (x < 18) {
                    x = 2;
                }
                else{
                    x = 3;
                }
                if(y % 2 == 0) {
                    y = 2;
                }
                else {
                    y = 3;
                }
            }
            
            if (y % 3 != 0) {
                type = 4;
                if (x < 18) {
                    x = 2;
                }
                else{
                    x = 3;
                }
                if(y % 2 == 0) {
                    y = 2;
                }
                else {
                    y = 3;
                }
        }
        else {
            //outer columns
            if (y % 3 == 0) { 
                type = 1;
                if (x < 18) {
                    x = 2;
                }
                else{
                    x = 3;
                }
                if(y % 2 == 0) {
                    y = 2;
                }
                else {
                    y = 3;
                }
            }
            if (y % 3 != 0){ 
                type=  3 ;
                if (x < 18) {
                    x = 2;
                }
                else{
                    x = 3;
                }
                if(y % 2 == 0) {
                    y = 2;
                }
                else {
                    y = 3;
                }
            }
        }        
    }
    else {
        //periphery
        if ( ( x       % 6 == 0 ) && (  x % 3      == 0) ) {
            type = 1;
        }
        if ( ((x - 10) % 6 == 0 ) && (  x % 3      == 0) ) {
            type = 2 ;
        }
        if ( ( x       % 6 == 0 ) && ( (x - 5) % 3 == 0) ) { 
            type= 3;
        if ( ((x - 10) % 6 == 0 ) && ( (x - 5) % 3 == 0) ) {
            type= 4;
        }
    }
}

*/

void logUnmask::logUnmaskEvent(long int evPU, short& metax, short& metay, short& pol, short& type) {
    //unmasking through LUT
    
    int x    = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    int y    = (short) ((evPU & ymask) >> yshift);
    pol  = ((short)((evPU & polmask) >> polshift)==0)?-1:1;        //+1 ON, -1 OFF
    //determining whether the address is in fovea or in periphery
}


void logUnmask::threadRelease() {
    //no istruction in threadInit
}


/*
void logUnmask::logUnmaskData(char* i_buffer, int i_sz) {
    //cout << "Size of the received packet to logUnmask : " << i_sz << endl;
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
            //logUnmask the data
            unsigned int part_1 = 0x00FF&i_buffer[j];
            unsigned int part_2 = 0x00FF&i_buffer[j+1];
            unsigned int part_3 = 0x00FF&i_buffer[j+2];
            unsigned int part_4 = 0x00FF&i_buffer[j+3];
            unsigned int blob = (part_1)|(part_2<<8);
            logUnmaskEvent(blob, cartX, cartY, polarity);
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


//----- end-of-file --- ( next line intentionally left blank ) ------------------

