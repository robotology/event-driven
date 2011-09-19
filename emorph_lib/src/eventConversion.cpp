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
 * @file unmask.cppy
 * @brief A class for unmasking the event (see the header unmask.h)
 */

#include <eventConversion.h>
#include <cassert>
#include <cstring>
#include <inttypes.h>

using namespace std;
using namespace yarp::os;


#define MAXVALUE 114748364 //4294967295
#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;

unmask::unmask(){ // : RateThread(UNMASKRATETHREAD){
    count = 0;
    verbosity  = false;
    dvsMode    = false;
    numKilledEvents = 0;
    lasttimestamp   = 0;
    validLeft  = false;
    validRight = false;
    eldesttimestamp = MAXVALUE;
    countEvent  = 0;
    countEvent2 = 0;
    minValue = 0;
    maxValue = 0;
    xmask = 0x000000fE;
    ymask = 0x00007f00;
    xmasklong = 0x000000fE;
    xmaskshort  = 0x00fE;
    ymaskshort  = 0x7f00;
    polmaskshort= 0x0001;
    yshift   = 8;
    yshift2  = 16,
    xshift   = 1;
    polshift = 0;
    polmask  = 0x00000001;
    camerashift = 15;
    cameramask  = 0x00008000;
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
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
    //uEvents = fopen("./uevents.txt","w");
}

/*
bool unmask::threadInit() {
    return true;
}
*/

unmask::~unmask() {
    delete[] buffer;
    delete[] timeBuffer;
    delete[] bufferRight;
    delete[] timeBufferRight;
}

void unmask::cleanEventBuffer() {
    memset(buffer,0,retinalSize*retinalSize*sizeof(int));
    memset(timeBuffer, 0, retinalSize * retinalSize * sizeof(unsigned long));
    minValue=0;
    maxValue=0;
}

int unmask::getMinValue() {
    return minValue;
}

int unmask::getMaxValue() {
    return maxValue;
}

unsigned long unmask::getLastTimestamp() {
    return lasttimestamp;
}

unsigned long unmask::getLastTimestampRight() {
    return lasttimestampright;
}

unsigned long unmask::getEldestTimeStamp() {
    return eldesttimestamp;
}

void unmask::setLastTimestamp(unsigned long value) {
    lasttimestamp = value;
}

int* unmask::getEventBuffer(bool camera) {
    if(camera)
        return buffer;
    else
        return bufferRight;
}

unsigned long* unmask::getTimeBuffer(bool camera) {
    if (camera)
        return timeBuffer;
    else
        return timeBufferRight;
}



/*
void unmask::run() {
    
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
    
}
*/


void unmask::unmaskData(char* i_buffer, int i_sz, AER_struct* output) {
    //cout << "Size of the received packet to unmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    AER_struct* iterEvent = output;
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //create a pointer that points every 4 bytes
    uint32_t* buf2 = (uint32_t*)i_buffer;
    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        if(!dvsMode) {
            // unmask the data ( first 4 byte blob, second 4 bytes timestamp)
            unsigned long blob = buf2[2 * evt];
            unsigned long t    = buf2[2 * evt + 1];
            //printf("0x%x 0x%x \n",blob, timestamp);
            
            // here we zero the higher two bytes of the address!!! Only lower 16bits used!
            blob &= 0xFFFF;
            unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
            timestamp = (unsigned long) t;
            //if(count % 100 == 0) {
            //    printf(" %d>%d,%d : %d : %d \n",blob,cartX,cartY,timestamp,camera);
            //}
            
        }
        else { //in DVS mode
            unsigned int blob, t;
            //buffer += 3; //checking a flag
            if((i_buffer[i + 3] & 0x80) == 0x80) {
                // timestamp bit 15 is one -> wrap;
                // now we need to increment the wrapAdd                
                //uses only 14 bit timestamps
                wrapAdd += 0x4000;                     
                //System.out.println("received wrap event, index:" + eventCounter + " wrapAdd: "+ wrapAdd);
                //NumberOfWrapEvents++;
            }
            else if  ((i_buffer[i + 3] & 0x40) == 0x40  ) {
                // timestamp bit 14 is one -> wrapAdd reset
                // this firmware version uses reset events to reset timestamps
                // write(file_desc,reset,1);//this.resetTimestamps();
                //            buffer_msg[0] = 6;
                //            write(file_desc,buffer_msg,1);
                wrapAdd = 0;
                //lasttimestamp = 0;
                // log.info("got reset event, timestamp " + (0xffff&((short)aeBuffer[i]&0xff | ((short)aeBuffer[i+1]&0xff)<<8)));
            }
            else 
            {
                //buffer -= 3;  //returning to the first byte of the event
                unsigned int part_1 = 0xFF & i_buffer[i];    //extracting the 1 byte        
                //buffer++;
                unsigned int part_2 = 0xFF & i_buffer[i + 1];  //extracting the 2 byte        
                //buffer++;
                unsigned int part_3 = 0xFF & i_buffer[i + 2];  //extracting the 3 byte
                //buffer++;
                unsigned int part_4 = 0xFF & i_buffer[i + 3];  //extracting the 4 byte
                //buffer++;
                
                blob      = (part_1)|(part_2<<8);          //16bits
                //printf("Bolob is%x \n",blob);
                polarity =0;
                unmaskEvent( blob, cartX, cartY, polarity); 
                short temp = cartX;
                cartX = -cartY;
                cartY = temp;
                //if((cartX<128 && cartY <128 && cartX >0 && cartY>0)) 
                //printf("cartX %d cartY%d polarity%d\n",cartX,cartY,polarity);
                //float timestamp = ((part_3)|(part_4<<8));
                t = ((part_3)|(part_4<<8)); //&0x7fff//        //16bits
                unsigned long tempT = t;
                t += wrapAdd;
                
                //if(i == 100){
                //printf("Saving in file \n");
                //printf("---> %08X %08X \n",blob,timestamp); 
                //fwrite(&sz, sizeof(int), 1, raw);
                //fwrite(buffer, 1, sz, raw);
                //}
                timestamp = (unsigned long) t;
                //if(!(cartX == 0 && cartY == 127 && t == 0))
                
                //printf("%04d %04d %08X \n",cartX, cartY ,t);
                //fprintf(uEvents,"%08X %08X \n",blob,tempT);
                
                //printf("lastTimeStamp %08X \n", timestamp);
                camera = 0;
            }             
        }
        

        // processing of the event
        cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        cartX = retinalSize - cartX;
            
        //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
        camera = camera + 1;        //camera: LEFT 0, RIGHT 1

        //adding a new event to the list
        iterEvent->x = cartX;
        iterEvent->y = cartY;
        iterEvent++;
    }
}

void unmask::unmaskData(char* i_buffer, int i_sz) {
    //cout << "Size of the received packet to unmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    //AER_struct sAER
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //create a pointer that points every 4 bytes
    uint32_t* buf2 = (uint32_t*)i_buffer;
    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        if(!dvsMode) {
            // unmask the data ( first 4 byte blob, second 4 bytes timestamp)
            unsigned long blob = buf2[2 * evt];
            unsigned long t    = buf2[2 * evt + 1];
            //printf("0x%x 0x%x \n",blob, timestamp);
            
            // here we zero the higher two bytes of the address!!! Only lower 16bits used!
            blob &= 0xFFFF;
            unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
            timestamp = (unsigned long) t;
            //if(count % 100 == 0) {
            //    printf(" %d>%d,%d : %d : %d \n",blob,cartX,cartY,timestamp,camera);
            //}
            
        }
        else { //in DVS mode
            unsigned int blob, t;
            //buffer += 3; //checking a flag
            if((i_buffer[i + 3] & 0x80) == 0x80) {
                // timestamp bit 15 is one -> wrap;
                // now we need to increment the wrapAdd                
                //uses only 14 bit timestamps
                wrapAdd += 0x4000;                     
                //System.out.println("received wrap event, index:" + eventCounter + " wrapAdd: "+ wrapAdd);
                //NumberOfWrapEvents++;
            }
            else if  ((i_buffer[i + 3] & 0x40) == 0x40  ) {
                // timestamp bit 14 is one -> wrapAdd reset
                // this firmware version uses reset events to reset timestamps
                // write(file_desc,reset,1);//this.resetTimestamps();
                //            buffer_msg[0] = 6;
                //            write(file_desc,buffer_msg,1);
                wrapAdd = 0;
                //lasttimestamp = 0;
                // log.info("got reset event, timestamp " + (0xffff&((short)aeBuffer[i]&0xff | ((short)aeBuffer[i+1]&0xff)<<8)));
            }
            else 
            {
                //buffer -= 3;  //returning to the first byte of the event
                unsigned int part_1 = 0xFF & i_buffer[i];    //extracting the 1 byte        
                //buffer++;
                unsigned int part_2 = 0xFF & i_buffer[i + 1];  //extracting the 2 byte        
                //buffer++;
                unsigned int part_3 = 0xFF & i_buffer[i + 2];  //extracting the 3 byte
                //buffer++;
                unsigned int part_4 = 0xFF & i_buffer[i + 3];  //extracting the 4 byte
                //buffer++;
                
                blob      = (part_1)|(part_2<<8);          //16bits
                //printf("Bolob is%x \n",blob);
                polarity =0;
                unmaskEvent( blob, cartX, cartY, polarity); 
                short temp = cartX;
                cartX = -cartY;
                cartY = temp;
                //if((cartX<128 && cartY <128 && cartX >0 && cartY>0)) 
                //printf("cartX %d cartY%d polarity%d\n",cartX,cartY,polarity);
                //float timestamp = ((part_3)|(part_4<<8));
                t = ((part_3)|(part_4<<8)); //&0x7fff//        //16bits
                unsigned long tempT = t;
                t += wrapAdd;
                
                //if(i == 100){
                //printf("Saving in file \n");
                //printf("---> %08X %08X \n",blob,timestamp); 
                //fwrite(&sz, sizeof(int), 1, raw);
                //fwrite(buffer, 1, sz, raw);
                //}
                timestamp = (unsigned long) t;
                //if(!(cartX == 0 && cartY == 127 && t == 0))
                
                //printf("%04d %04d %08X \n",cartX, cartY ,t);
                //fprintf(uEvents,"%08X %08X \n",blob,tempT);
                
                //printf("lastTimeStamp %08X \n", timestamp);
                camera = 0;
            }             
        }
        

        // processing of the event
        cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        cartX = retinalSize - cartX;
            
        //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
        camera = camera + 1;        //camera: LEFT 0, RIGHT 1
    }
}

void unmask::resetTimestamps() {
    for (int i=0 ; i<retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
        timeBufferRight[i] = 0;
    }
    lasttimestamp = 0;
    lasttimestampright = 0;
    verbosity = true;
}

void unmask::resetTimestampLeft() {
    for (int i=0 ; i<retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
    }
    lasttimestamp = 0;
}

void unmask::resetTimestampRight() {
    for (int i=0 ; i<retinalSize * retinalSize; i++){
        timeBufferRight[i] = 0;
    }
    lasttimestampright = 0;
}


/**
 * @brief masking of a 32bit address event with camera information
 */
void unmask::maskEvent( short x, short y, short pol, short camera,unsigned long& evPU ) {
    //y = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    //x = (short) ((evPU & ymask) >> yshift);
    //pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    //camera = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
    pol = pol==
    eventPU = y << xshift + x << yshift + pol << polshif 

}


/**
 * @brief unmasking of a 16bit address event with camera information
 */
void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol, short& camera) {
    y = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    x = (short) ((evPU & ymask) >> yshift);
    pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    camera = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
}

/**
 * @brief unmasking of a 16bit address event with no information about camera
 */
void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol) {
    y =       (short) (retinalSize - 1) - (short)((evPU & xmaskshort) >> xshift);
    x =       (short) ((evPU & ymaskshort)      >> yshift);
    pol =    ((short) ((evPU & polmaskshort)    >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    //printf("2evPU%x polmask%x polshift%x \n",evPU, polmask,polshift);         
}

/**
 * @brief unmasking of 32bit address event
 */
void unmask::unmaskEvent(long int evPU, short& x, short& y, short& pol, short& camera) {
    x = (short)(retinalSize-1) - (short)((evPU & xmask) >> xshift);
    y = (short) ((evPU & ymask) >> yshift);
    pol = ((short)((evPU & polmask) >> polshift)==0)?-1:1;        //+1 ON, -1 OFF
    camera = ((short)(evPU & cameramask) >> camerashift);	//0 LEFT, 1 RIGHT
}


/*
void unmask::threadRelease() {
    //no istruction in threadInit
}
*/

//----- end-of-file --- ( next line intentionally left blank ) ------------------

