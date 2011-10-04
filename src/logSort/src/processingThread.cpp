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
 * @file processingThread.cpp
 * @brief Implementation of the thread that process the events in the collected buffers (see header plotterThread.h)
 */


#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cstdio>
#include <cxcore.h>


#include <iCub/processingThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 30 
#define MAXLIMIT 4000000000
#define MINLIMIT   10000000
//#define retinalSize 128

processingThread::processingThread() : RateThread(THRATE) {
    synchronised = false;
    count=0;
    retinalSize =  24;
    countEM1 = 0;
    countEM2 = 0;
    countEM3 = 0;
    countEM4 = 0;
    bufferEM1 = 0;
    bufferEM2 = 0;
    bufferEM3 = 0;
    bufferEM4 = 0;
    cartEM          = new aer[24 * 24];
    pEM             = new aer[24 * 24];
    memset(cartEM,  0, 24 * 24 * sizeof(aer));
    memset(pEM,     0, 24 * 24 * sizeof(aer));
}

processingThread::~processingThread() {
    printf("freeing memory in collector");
    delete[] cartEM;
    delete[] pEM;
}

bool processingThread::threadInit() {
    printf("\n starting the thread.... \n");    
    return true;
}

void processingThread::interrupt() {
}

void processingThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string processingThread::getName(const char* p) {
  string str(name);
  str.append(p);
  return str;
}

void processingThread::getEM(aer** pointerEM, int* dimEM) {
  //printf("counted EM %d \n", countEM);
  //*pointerEM = bufferEM1;
  //*dimEM = countEM1 + countEM2 + countEM3 + countEM4;
  *dimEM = 0;
  
  for (int pos = 0; pos < 24 * 24; pos++){
    if(cartEM[pos].address != 0) {
      //printf("cartEM[pos] %08x \n",cartEM[pos].address);
      pEM[*dimEM] = cartEM[pos];
      (*dimEM)++;            
    }
  }
  *pointerEM = pEM;
  //delete[] pEM;
  memset(cartEM,0, 24 * 24 * sizeof(aer));
}

void processingThread::setEM(aer* bEM1, aer* bEM2, aer* bEM3, aer* bEM4) {
  bufferEM1 = bEM1;
  bufferEM2 = bEM2;
  bufferEM3 = bEM3;
  bufferEM4 = bEM4;
}

void processingThread::addBufferEM(aer* event){
    // extracts coordinate in the output image size
    unsigned long current_blob      = event->address;
    unsigned long current_timestamp = event->timestamp;
    unsigned int current_value      = (current_blob & 0xffff0000) >> 16;
   
    unsigned short x = (current_blob & 0x00FE) >> 1;
    unsigned short y = (current_blob & 0xFF00) >> 8;
    //printf("current_blob %08x position x %d y %d value %d \n",current_blob, x, y, current_value);
         
    // compare with the  event was inside
    int position = x + y * 24;
    if((cartEM[position].address == 0)&&(cartEM[position].timestamp == 0)){
        cartEM[position].address   = event->address;
        cartEM[position].timestamp = event->timestamp;
    }
    else {
        //taking the mean value of the two events;
        unsigned long old_blob      = cartEM[position].address;
        unsigned long old_timestamp = cartEM[position].timestamp;
        unsigned int  old_value     = (old_blob & 0xffff0000) >> 16;
        unsigned int  new_value     = (int)floor((current_value + old_value)/2);
        printf("old_blob %x old_value %d current_value %d  new_value %d \n",old_blob, old_value,current_value, new_value);
        unsigned long new_timestamp = (old_timestamp + current_timestamp) >> 1;
        unsigned long new_blob      = (new_value << 16) & old_blob; 
        cartEM[position].address    = new_blob;
        cartEM[position].timestamp  = new_timestamp;
    }
}


unsigned long processingThread::look4opposite(aer* buffer,int initPos, int countTOT){
    unsigned long targetBlob;
    unsigned long blob = buffer[initPos].address;
    if (blob % 2 == 0) {
      // high event looking for low
      targetBlob = blob + 1;
    }
    else {
      return 0;
      //low event looking for high
      targetBlob = blob - 1;
    }
    bool found = false;
    int i;
    for (i = initPos + 1; (i< countTOT) && (!found); i++) {
        if(buffer[i].address == targetBlob) {
            found = true;
        }        
    }
    if(found) {
        return buffer[i].timestamp;
    }
    else {
        return buffer[initPos].timestamp;
    }
}


void processingThread::run() {
    count++;
    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM1
    unsigned long maxdiff = 0, mindiff = 0xFFFFFFFF;
    for(int i = 0; i< countEM1 ; i++){
        //printf("analysing EM1 position %d \n", countEM1);
        unsigned long blob = bufferEM1[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM1[i].timestamp;
        timestampFound = look4opposite(bufferEM1,i,countEM1);
        long diff =  timestampFound - timestamp;
        unsigned long absdiff = std::abs(diff);
        if(absdiff == 0) {
            continue;
        }
        if (absdiff > maxdiff){ maxdiff = absdiff; }
        if (absdiff < mindiff){ mindiff = absdiff; }
        int numbits = 256;        
        double max = MAXLIMIT;
        double min = MINLIMIT;
        int value = floor(((double)absdiff /(max - min)) * 256.0);
        if(value > 255) {
            value = 255;
        }
        printf("    buffer.address %08x   ",bufferEM1[i].address);
        printf("value %lu binaryvalue %d   \n ", absdiff,value);
        printf("shifted16 %08x \n",value<<16 );
        bufferEM1[i].address = bufferEM1[i].address | (value<<16);
        printf(" buffer.address %08x \n",bufferEM1[i].address);
        addBufferEM(&bufferEM1[i]);
        
        //printf("look4opposite EM1 %d: %08x %08x > %08x %d \n",i,blob, timestamp,timestampFound, absdiff);
        //if(absdiff != 0) {
        //    unsigned short x    = ((blob & 0x00FF) >> 1);
        //    unsigned short y    = ((blob & 0x7F00) >> 8);
        //    if(cartEM[x + y * retinalSize]= 0) {
        //        cartEM[x + y * retinalSize] = absdiff;
        //    }
        //    else {
        //        cartEM[x + y * retinalSize] = (cartEM[x + y * retinalSize] + absdiff)/2; 
         //   }
         //   }
    }

    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM2    
    for(int i = 0; i< countEM2 ; i++){
        //printf("analysing EM2 position %d \n", countEM2);
        unsigned long blob = bufferEM2[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM2[i].timestamp;
        timestampFound = look4opposite(bufferEM2,i,countEM2);
        long diff =  timestampFound - timestamp;
        unsigned long absdiff = std::abs(diff);
        if(absdiff == 0) {
            continue;
        }
        if (absdiff > maxdiff){ maxdiff = absdiff; }
        if (absdiff < mindiff){ mindiff = absdiff; }

        int numbits = 256;        
        double max = MAXLIMIT;
        double min = MINLIMIT;
        int value = floor(((double)absdiff /(max - min)) * 256.0);
        if(value > 255) {
            value = 255;
        }
        printf("    buffer.address %08x   ",bufferEM2[i].address);
        printf("value %lu binaryvalue %d    \n ", absdiff,value);
        bufferEM2[i].address = (bufferEM2[i].address & 0x0000FFFF) | (value<<16);
        printf(" buffer.address %08x \n",bufferEM1[i].address);
        addBufferEM(&bufferEM2[i]);
    }
    

    //printf(" - - - - - - - - - - - - - - \n");
    // searching the couples in EM3
    
    for(int i = 0; i< countEM3 ; i++){
        //printf("analysing EM3 position %d \n", countEM3);
        unsigned long blob = bufferEM3[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM3[i].timestamp;
        timestampFound = look4opposite(bufferEM3,i,countEM3);
        long diff =  timestampFound - timestamp;
        unsigned long absdiff = std::abs(diff);
        if(absdiff == 0) {
            continue;
        }
        if (absdiff > maxdiff){ maxdiff = absdiff; }
        if (absdiff < mindiff){ mindiff = absdiff; }

        int numbits = 256;        
        double max = MAXLIMIT;
        double min = MINLIMIT;
        int value = floor(((double)absdiff /(max - min)) * 256.0);
        if(value > 255) {
            value = 255;
        }
        printf("    buffer.address %08x   ",bufferEM3[i].address);
        printf("value %lu binaryvalue %d     \n", absdiff,value);
        bufferEM3[i].address = (bufferEM3[i].address & 0x0000FFF) | (value<<16);
        printf(" buffer.address %08x \n",bufferEM3[i].address);
        addBufferEM(&bufferEM3[i]);
    }    
    //printf(" - - - - - - - - - - - - - - \n");
    

    // searching the couples in EM4
    for(int i = 0; i< countEM4 ; i++){
        //printf("analysing EM4 position %d \n", countEM4);
        unsigned long blob = bufferEM4[i].address;
        unsigned long timestampFound;
        unsigned long timestamp = bufferEM4[i].timestamp;
        timestampFound = look4opposite(bufferEM4,i,countEM4);
        long diff =  timestampFound - timestamp;
        unsigned long absdiff = std::abs(diff);
        if(absdiff == 0) {
            continue;
        }
        int numbits = 256;        
        double max = MAXLIMIT;
        double min = MINLIMIT;
        int value = floor(((double)absdiff /(max - min)) * 256.0);
        if(value > 255) {
            value = 255;
        }
        if (absdiff > maxdiff){ maxdiff = absdiff; }
        if (absdiff < mindiff){ mindiff = absdiff; }
        printf("    buffer.address %08x   ",bufferEM1[i].address);
        printf("value %lu binaryvalue %d   \n ", absdiff,value);
        bufferEM4[i].address = (bufferEM4[i].address & 0x0000FFFF) | (value<<16);
        printf(" buffer.address %08x \n",bufferEM1[i].address);
        addBufferEM(&bufferEM4[i]);
    }  
    if(maxdiff != 0)
      printf("limits:  maxdiff  %lu \n", maxdiff);
    if(mindiff != 0xFFFFFFFF)
      printf("limits: mindiff %lu \n", mindiff);
}




void processingThread::threadRelease() {
  printf("processingThread: releasing \n");  

}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
