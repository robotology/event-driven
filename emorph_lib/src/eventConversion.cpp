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
 * @file eventConversion.cpp
 * @brief A class for conversion of events event (see the eventConversion.h)
 */

#include <iCub/emorph/eventConversion.h>

#include <cassert>
#include <cstring>
//#include <inttypes.h>

using namespace std;
using namespace yarp::os;
using namespace emorph::ecodec;

#define MAXVALUE 114748364 //4294967295
#define maxPosEvent 10000
#define responseGradient 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;

#define VERBOSE

unmask::unmask() { 
    retinalSize = 128;
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

#ifdef VERBOSE
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
    uEvents = fopen("./eventConversion.uevents.txt","w+");
#endif
}

unmask::unmask(int rSize) { // : RateThread(UNMASKRATETHREAD){
    retinalSize = rSize;
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

#ifdef VERBOSE
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
    uEvents = fopen("./eventConvesion.uevents.txt","w+");
#endif
}

unmask::~unmask() {
#ifdef VERBOSE
    fclose(uEvents);
#endif
    

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

void unmask::updateImage(AddressEvent* ptr) {
    // ********** extract properties **********************************
    cartY     = ptr->getX();
    cartX     = ptr->getY();
    camera    = ptr->getChannel();
    polarity  = ptr->getPolarity();
    //printf("blob %04d %04d %04d %04d \n", cartX, cartY, camera, polarity);
    
    timestamp = lastRecTimestamp;

    // // // // ************** created the images ***************************************
    // // // //correcting the orientation of the camera
    cartY = retinalSize - cartY - 1;   //corrected the output of the camera (flipped the image along y axis)
    
    // // // if((cartX < 0)||(cartX > retinalSize)){
    // // //     cartX = 0;
    // // // }
    // // // if((cartY < 0)||(cartY> retinalSize)){
    // // //     cartY = 0;
    // // // }
    cartX = retinalSize - cartX;
    
    // // // //if(cartX!=0) {
    // // // //    printf("retinalSize %d cartX %d cartY %d camera %d \n",retinalSize,cartX, cartY,camera);
    // // // //}
    // // // //printf("lastTimeStamp %08X \n", lasttimestamp);

    // camera is unmasked as left 0, right -1. It is converted in left 1, right 0
    // camera = camera + 1;
    // // // //printf("Camera %d polarity %d  \n", camera, polarity);
    // now camera: LEFT 1, RIGHT 0
    
    if(camera) {  // ---- camera left -------------------          
        if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
            validLeft =  true;
        }
        
        if(timestamp > lasttimestamp) {
            lasttimestamp = timestamp;
        }
             
        if(timeBuffer[cartX + cartY * retinalSize] < timestamp) {
            if(polarity > 0) {
                buffer[cartX + cartY * retinalSize]     += responseGradient;
                timeBuffer[cartX + cartY * retinalSize] = timestamp;
                
                if(buffer[cartX + cartY * retinalSize] > 127) {
                    buffer[cartX + cartY * retinalSize] = 127;
                }                
            }
            else /*if(polarity < 0)*/ {
                buffer[cartX + cartY * retinalSize] -= responseGradient;
                timeBuffer[cartX + cartY * retinalSize] = timestamp;
                
                if (buffer[cartX + cartY * retinalSize] < -127) {
                    buffer[cartX + cartY * retinalSize] = -127;
                }
            }
        }           
    }
    else { // --------  camera right ------------------------
        
        if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
            validRight =  true;
        }
        
        if( timestamp > lasttimestampright){
            lasttimestampright = timestamp;
        }    
        
        if (timeBufferRight[cartX + cartY * retinalSize] < timestamp) {
            if(polarity > 0) {
                bufferRight[cartX + cartY * retinalSize] += responseGradient;
                timeBufferRight[cartX + cartY * retinalSize] = timestamp;
                
                if(bufferRight[cartX + cartY * retinalSize] > 127) {
                    bufferRight[cartX + cartY * retinalSize] = 127;
                }
            }
            else /*if(polarity < 0)*/ {
                bufferRight[cartX + cartY * retinalSize] -= responseGradient;
                timeBufferRight[cartX + cartY * retinalSize] = timestamp;
                
                if (bufferRight[cartX + cartY * retinalSize] < -127) {
                    bufferRight[cartX + cartY * retinalSize] = -127;
                }
            }
        }
    }
}

void unmask::unmaskData(Bottle* packets) {
    //AER_struct sAER
    count++;
    int num_events = packets->size();
    //printf("packet size %d \n", num_events);
    
    //if(dvsMode) {
    //    num_events = i_sz / 4;    // pointing to events made of 4 bytes
    //}
    //else {
    //    num_events = i_sz / 8;    // pointing to events made of 8 bytes
    //}
    //cout << "Number of the received events : " << num_events<< endl;
    
    //create a pointer that points every 4 bytes and 2 bytes (DVS mode) 
    //uint32_t* buf2 = (uint32_t*)i_buffer;
    //uint16_t* buf1 = (uint16_t*)i_buffer;
    //unsigned long timestamp;
          
    //eldesttimestamp = 0;
    int i = 0;
    eEventQueue q;  
    if(packets->isNull()) {
        printf("null bottle \n");
    }
    else {
#ifdef VERBOSE
        fprintf(uEvents,"dim %d \n", packets->size());
        string str;
        int chksum;
        //printf("is Null? %d \n", packets->isNull());
        for (int j = 0; j < packets->size(); j++) {
            //printf(">%08x  \n", (unsigned int) packets->get(j).asInt());
            fprintf(uEvents, ">%08x  \n", (unsigned int) packets->get(j).asInt());
            chksum = packets->get(i).asInt() % 255;
            str[i] = (char) chksum;
        }
        //printf("%s \n", packets->toString().c_str());
        fprintf(uEvents,"chksum: %s \n", str.c_str());
        fprintf(uEvents,"--- \n");
#endif  
        //-- decoding the packet -------
        printf("trying to decode the packet \n");
        if(eEvent::decode(*packets,q)) {
            //printf("pointer %08X \n",  &q);
            //printf("deque size %d \n \n", (int) q.size());
            int dequeSize = q.size();
#ifdef VERBOSE
            fprintf(uEvents, " dim : %d \n", dequeSize);
#endif
            for (int evt = 0; evt < dequeSize; evt++) {
                //printf("evt : %d \n", evt);                
                if(q[evt] != 0) {                    
                    //********** extracting the event information **********************
                    // to identify the type of the packet
                    // user can rely on the getType() method
                    if (q[evt]->getType()=="AE") {
                        //printf("address event \n");
                        // identified an  address event
                        AddressEvent* ptr=dynamic_cast<AddressEvent*>(q[evt]);
                        if(ptr->isValid()) { 
#ifdef VERBOSE                      
                            fprintf(uEvents,"content: %s \n", ptr->getContent().toString().c_str());            
#endif
                            //printf("%d %d %d %d \n",ptr->getX(), ptr->getY(), ptr->getChannel(), ptr->getPolarity());
                            //if((ptr->getX() == 0) && (ptr->getY() == 0) && (ptr->getChannel()==0) && (ptr->getPolarity() == 0)) {
                            //    printf("null address \n");
                            //}
                            
                            updateImage(ptr);                           
                        }
                    }
                    else if(q[evt]->getType()=="TS") {
                        //printf("timestamp \n");
                        TimeStamp* ptr=dynamic_cast<TimeStamp*>(q[evt]);
#ifdef VERBOSE
                        if(ptr->isValid()) {                       
                            fprintf(uEvents,"content: %08x \n", (unsigned int) ptr->getStamp());    
                        }
#endif
                        //identified an time stamp event
                        lastRecTimestamp = (unsigned int) ptr->getStamp();
                        //printf("lastTimestamp Received %08x \n", lastRecTimestamp);
                    }
                    else {
                        printf("not recognized");
                    }
                }
                else {
                    printf("null q[evt] \n");
                }
            } //end of for   
        } // end eEvent::decode
        else {
            printf("ERROR in decoding; eEvent::decode failed  \n");
        }    
    } // end else packets->isNull;    
}

void unmask::unmaskData(Bottle* packets,eEventQueue* q ) {
    //AER_struct sAER
    count++;
    int num_events = packets->size();
    //printf("packet size %d \n", num_events);
    
    //if(dvsMode) {
    //    num_events = i_sz / 4;    // pointing to events made of 4 bytes
    //}
    //else {
    //    num_events = i_sz / 8;    // pointing to events made of 8 bytes
    //}
    //cout << "Number of the received events : " << num_events<< endl;
    
    //create a pointer that points every 4 bytes and 2 bytes (DVS mode) 
    //uint32_t* buf2 = (uint32_t*)i_buffer;
    //uint16_t* buf1 = (uint16_t*)i_buffer;
    //unsigned long timestamp;
          
    //eldesttimestamp = 0;
    int i = 0;
      
    if(packets->isNull()) {
        printf("null bottle \n");
    }
    else {
        //printf("unmasking packet dimension %d \n", packets->size());
#ifdef VERBOSE
        fprintf(uEvents,"dim %d \n", packets->size());
        string str;
        int chksum;
        for (int j = 0; j < packets->size(); j++) {
            //printf(">%08x  \n", (unsigned int) packets->get(j).asInt());
            fprintf(uEvents, ">%08x  \n", (unsigned int) packets->get(j).asInt());
            chksum = packets->get(i).asInt() % 255;
            str[i] = (char) chksum;
        }
        //printf("%s \n", packets->toString().c_str());
        fprintf(uEvents,"chksum: %s \n", str.c_str());
        fprintf(uEvents,"--- \n");
        
#endif  
        //-- decoding the packet -------
        //printf("preparing the decoding procedure  \n");
        eEvent::decode(*packets,*q);   
        //printf("success in decoding the events \n");
    }
}

int unmask::unmaskData(char* i_buffer, int i_sz, AER_struct* output) {
    //cout << "Size of the received packet to unmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    AER_struct* iterEvent = output;
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //create a pointer that points every 4 bytes
    u32* buf2 = (u32*)i_buffer;
    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        
        // Beware : change in 06/02/12
        // before : unmask the data ( first 4 byte blob, second 4 bytes timestamp)
        // now    : unmask the data ( first 4 byte timestamp, second 4 bytes blob)
        unsigned long t    = buf2[2 * evt];
        unsigned long blob = buf2[2 * evt + 1];
        if ((t != 0) || (blob != 0)) {
            //printf("0x%x 0x%x ",blob, timestamp);
        
            i = i + 1;
            // here we zero the higher two bytes of the address!!! Only lower 16bits used!
            blob &= 0xFFFF;
            unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
            timestamp = (unsigned long) t;
            
            // processing of the event
            cartY = retinalSize - cartY - 1;   //corrected the output of the camera (flipped the image along y axis)
            cartX = retinalSize - cartX - 1;
            
            

            //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
            camera = camera + 1;        //camera: LEFT 0, RIGHT 1
            //printf("camera %d   \n", camera);
            
            //adding a new event to the list
            iterEvent->x   = cartX;
            iterEvent->y   = cartY;
            iterEvent->pol = polarity;
            iterEvent->cam = camera;
            iterEvent->ts  = timestamp;
            iterEvent++;
            
            if(timestamp > lasttimestamp) {
                lasttimestamp = timestamp;
            }
        }
    }
    return i;
}


void unmask::unmaskData(char* i_buffer, int i_sz, aer* output) {
    //cout << "Size of the received packet to unmask : " << i_sz / 8<< endl;
    //printf("pointer 0x%x ",i_buffer);
    aer* iterAer = output;
    count++;
    //assert(num_events % 8 == 0);
    int num_events = i_sz / 8;
    //create a pointer that points every 4 bytes
    u32* buf2 = (u32*)i_buffer;
    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {

        // unmask the data ( first 4 byte blob, second 4 bytes timestamp)
        unsigned long blob = buf2[2 * evt];
        unsigned long t    = buf2[2 * evt + 1];
        //printf("0x%x 0x%x \n",blob, timestamp);
        
        // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        blob &= 0xFFFF;
        unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
        timestamp = (unsigned long) t;
        

        // processing of the event
        cartY = retinalSize - cartY;   //corrected the output of the camera (flipped the image along y axis)
        cartX = retinalSize - cartX;
            
        //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
        camera = camera + 1;        //camera: LEFT 0, RIGHT 1

        //adding a new event to the list
        
        u32 address = polarity;
        address     = address | cartX  << xshift;
        address     = address | cartY  << yshift;
        address     = address | camera << camerashift;
 
        iterAer->address   = address;
        iterAer->timestamp = timestamp;
        iterAer++;

        if(timestamp > lasttimestamp) {
            lasttimestamp = timestamp;
        }
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
    u32* buf2 = (u32*)i_buffer;
    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        if(!dvsMode) {
            // before 7/2/12 unmask the data ( first 4 byte blob, second 4 bytes timestamp)
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

        if(timestamp > lasttimestamp) {
            lasttimestamp = timestamp;
        }
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
    //evPU = y << xshift + x << yshift + pol << polshift;
    evPU = pol;
    evPU = evPU | (x << xshift);
    evPU = evPU | (y << yshift);
    evPU = evPU | (camera << camerashift);
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

