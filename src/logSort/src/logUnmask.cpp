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


#define MAXVALUE 114748364 //4294967295
#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;
#define SIZE_OF_EVENT 128


logUnmask::logUnmask() : RateThread(UNMASKRATETHREAD){
    count = 0;
    countCD = 0;
    countEM = 0;
    countIF = 0;
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

    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);
    bufferCD = (aer *)  malloc(monBufSize_b);
    if ( bufferCD == NULL ) {
        printf("pmon malloc failed \n");
    }
    else {
        printf("bufferCD successfully created \n");
    }
    bufferEM = (aer *)  malloc(monBufSize_b);
    if ( bufferEM == NULL ) {
        printf("pmon malloc failed \n");
    }
    else {
        printf("bufferEM successfully created \n");
    }

    bufferIF = (aer *)  malloc(monBufSize_b);
    if ( bufferIF == NULL ) {
        printf("pmon malloc failed \n");
    }
    else {
        printf("bufferIF successfully created \n");
    }

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the logUnmasked_buffer
    //uEvents = fopen("./uevents.txt","w");

    logChip_LUT = new feature[648]; // all the position in the logchip times 4 features 
}

logUnmask::~logUnmask() {
    printf("logUnmask : freeing memory allocated by buffers \n");
    delete[] buffer;
    delete[] timeBuffer;
    delete[] bufferRight;
    delete[] timeBufferRight;
    delete[] logChip_LUT;
    printf("logUnmask : freeing memory allocated by event buffers \n");
    delete bufferCD;
    delete bufferIF;
    delete bufferEM;
}

bool logUnmask::threadInit() {
    // initialising the logChip_LUT
    fout = fopen("dump.txt", "w");
    
    /*  // structure of the LUT for logpolar Chip
    // type, metaX, metaY, polarity
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
            
            if( ((x - 2) % 6 == 0) || ((x - 3) % 6 == 0)){
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
            
            else if (((x - 6) % 12 == 0) && ( 
                                             ((y >= 12) || (y <= 6))
                                             &&
                                             ((x <= 12) || (x >= 24))
                                             )
                     ){
                // IFON
                type  = 5;
                metax = (int) (x - 6) / 6;
                metay = (y % 2 == 0) ? y / 3 : (y / 3) - 1;
                pol = 1;                                  
            } // IFON
            
            else if (((x - 7) % 12 == 0) && ( 
                                             ((y >= 12) || (y <= 6))
                                             &&
                                             ((x <= 12) || (x >= 24))
                                             )
                     ){
                // IFOFF
                type  = 5;
                metax = (int) (x - 6) / 6;
                metay = (y % 2 == 0) ? y / 3 : (y / 3) - 1;
                pol = 0;                                  
            } // IFOFF

            // EM 
            else if((y >= 6) && (y < 12) && (x >= 12) && (x < 24)) {
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
                else if ( ((x - 1 ) % 6 == 0 ) && (  y % 3      == 0) ) {
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
                else if ( ((x - 10) % 6 == 0 ) && (  y % 3      == 0) ) {
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
                else if ( ((x - 11) % 6 == 0 ) && (  y % 3      == 0) ) {
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
                else if ( ( x       % 6 == 0 ) && ( (y - 5) % 3 == 0) ) { 
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
                else if ( ((x - 1)       % 6 == 0 ) && ( (y - 5) % 3 == 0) ) { 
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
                else if ( ((x - 10) % 6 == 0 ) && ( (y - 5) % 3 == 0) ) {
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
                else if ( ((x - 11) % 6 == 0 ) && ( (y - 5) % 3 == 0) ) {
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
            
            // saving the extracted information in a LUT respecting the order 
            // type, metax, metay, polarity
            logChip_LUT[y * 36 + x][0] = type ; logChip_LUT[y * 36 + x][1] = metax ; logChip_LUT[y * 36 + x][2] = metay ; logChip_LUT[y * 36 + x][3] = pol;
            
            //if((true) && ( y == 11)) {
            //    printf(" %d %d > %d %d %d %d \n", x, y, metax, metay, pol, type);
            //}
        }
    }
    return true;
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

void logUnmask::getCD(aer* pointerCD, int* dimCD) {
    //printf("counted CD %d \n", countCD);
    pointerCD = bufferCD;
    dimCD = &countCD;
}

void logUnmask::getIF(aer* pointerIF, int* dimCD) {
    //printf("counted IF %d \n", countIF);
    pointerIF = bufferIF;
}

void logUnmask::getEM(aer* pointerEM, int* dimCD) {
    //printf("counted EM %d \n", countEM);
    pointerEM = bufferEM;
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
    //printf("logUnmakData: unmasking %d  \n", num_events);
    //create a pointer that points every 4 bytes
    uint32_t* buf2 = (uint32_t*)i_buffer;
    //eldesttimestamp = 0;

    for (int evt = 0; evt < num_events; evt++) {
        
        // logUnmask the data ( first 4 byte blob, second 4 bytes timestamp)
        unsigned long blob      = buf2[2 * evt];
        unsigned long timestamp = buf2[2 * evt + 1];
        
                
        // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        //blob &= 0xFFFF;
        type = -1;
  
        bool save = false;
        
        //printf("%08X %08X \n",blob,timestamp);
        if (save) {
            fprintf(fout,"%08X %08X\n",blob,timestamp); 
            //fout<<hex<<a<<" "<<hex<<t<<endl;
        }
        logUnmaskEvent((unsigned int) blob, cartX, cartY, polarity, type);
        
        

        //if(count % 100 == 0) {
        //    printf(" %d>%d,%d : %d : %d \n",blob,cartX,cartY,timestamp,camera);
        //}
        cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        cartX = retinalSize - cartX;

        struct aer* temp;
        
        switch (type) {
            case 0:{ //CD
                //printf("Unmasked CD \n");
                temp = &bufferCD[countCD];
                temp->address   = blob;
                temp->timestamp = timestamp;
                countCD++;
                
            }
            break;
            case 1:{ //EM1
                //printf("Unmasked EM1 \n");
                countEM++;
            }
            break;
            case 2:{ //EM2
                //printf("Unmasked EM2 \n");
                countEM++;
            }
            break;
            case 3:{ //EM3
                //printf("Unmasked EM3 \n");
                countEM++;
            }
            break;
            case 4:{ //EM4
                //printf("Unmasked EM4 \n");
                countEM++;
            }
            break;
            case 5:{ //IF
                //printf("Unmasked IF \n");
                countIF++;
            }
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


void logUnmask::logUnmaskEvent(unsigned int evPU, short& metax, short& metay, short& pol, short& type) {
    int y = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    int x = (short) ((evPU & ymask) >> yshift);
    // 2.extractiong features 
    int position =  y * 36 + x;
    
    //feature* pFeature = logChip_LUT;
    //pFeature += position;
    //short* pcols = pFeature;
    //type  = pcols;
    //pcols++;
    //metax = pcols;
    //pcols++;
    //metay = pcols;
    //pcols++;
    //pol   = pcols;    

    type  = logChip_LUT[position][0];
    metax = logChip_LUT[position][1];
    metay = logChip_LUT[position][2];
    pol   = logChip_LUT[position][3];
    
    //printf("unmasked event %d %d %d %d \n",type, metax, metay, pol);

    //pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    //type = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
}

void logUnmask::logUnmaskEvent(long int evPU, short& metax, short& metay, short& pol, short& type) {
    // unmasking through LUT    
    // 1.determining the position in the LUT
    int x    = (short) (retinalSize-1) - (short)((evPU & xmask) >> xshift);
    int y    = (short) ((evPU & ymask) >> yshift);
    // 2.extractiong features 
    int position =  y * 36 + x;
    
    //feature* pFeature = logChip_LUT;
    //pFeature += position;
    //short* pcols = pFeature;
    //type  = pcols;
    //pcols++;
    //metax = pcols;
    //pcols++;
    //metay = pcols;
    //pcols++;
    //pol   = pcols;    

    type  = logChip_LUT[position][0];
    metax = logChip_LUT[position][1];
    metay = logChip_LUT[position][2];
    pol   = logChip_LUT[position][3];
    
    //printf("unmasked event %d %d %d %d \n",type, metax, metay, pol);
}


void logUnmask::threadRelease() {
    //no istruction in threadInit
    printf("closing the dumping file");
    fclose(fout);
    printf("successfully close the dumping file \n");
}




//----- end-of-file --- ( next line intentionally left blank ) ------------------

