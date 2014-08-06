// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file unmask.cpp
 * @brief A class for unmasking the event (see the header unmask.h)
 */


#include <math.h>
#include <cassert>
//#include <inttypes.h>

//withinProject includes
#include "unmask.h"

using namespace std;
using namespace yarp::os;
using namespace emorph::ecodec;

#define MAXVALUE 114748364 //4294967295
#define maxPosEvent 10000
//#define responseGradient 127  //default : 127
#define minKillThres 1000
#define UNMASKRATETHREAD 1
#define constInterval 100000;

//#define VERBOSE

//TODO : remove inheretance of this class from ratethread. No reason to be ratethread
unmask::unmask() : RateThread(UNMASKRATETHREAD){
    count         = 0;
    countCLE      = 0;
    countCLERight = 0;
    verb       = false;
    dvsMode    = false;
    asvMode    = false; //TODO : make this variable a parameter in the command line
    validLeft  = false;
    validRight = false;
    temp1      = true;

    numKilledEvents = 0;
    lasttimestamp = 0;
    eldesttimestamp = MAXVALUE;
    countEvent  = 0;
    countEvent2 = 0;
    minValue    = 0;
    maxValue    = 0;
    xmask       = 0x000000fE;
    ymask       = 0x00007f00;
    xmasklong   = 0x000000fE;
    polmask     = 0x00000001;
    cameramask  = 0x8000;
    xmaskshort  = 0x00fE;
    ymaskshort  = 0x7f00;
    polmaskshort= 0x0001;
    cameramaskshort = 0x8000;
    yshift      = 8;
    yshift2     = 16,
    xshift      = 1;
    polshift    = 0;
    camerashift = 15;
    retinalSize = 128;
    responseGradient = 127;

    /*fifoEvent=new int[maxPosEvent];
    memset(fifoEvent,0,maxPosEvent*sizeof(int));
    fifoEvent_temp=new int[maxPosEvent];
    memset(fifoEvent_temp,0,maxPosEvent*sizeof(int));
    fifoEvent_temp2=new int[maxPosEvent];
    memset(fifoEvent_temp2,0,maxPosEvent*sizeof(int));
    */

    wrapAdd = 0;
    //fopen_s(&fp,"events.txt", "w"); //Use the unmasked_buffer
//    uEvents    = fopen("cartesianFrameCollector.uevents.txt","w+");
//    maskEvents = fopen("cartesianFrameCollector.maskEvents.txt","w+");
}

bool unmask::threadInit() {
    buffer         = new int[retinalSize*retinalSize];
    bufferRight    = new int[retinalSize*retinalSize];
    timeBuffer     = new unsigned long[retinalSize*retinalSize];
    timeBufferRight= new unsigned long[retinalSize*retinalSize];

    memset(buffer ,         0,retinalSize*retinalSize*sizeof(int));
    memset(bufferRight,     0,retinalSize*retinalSize*sizeof(int));
    memset(timeBuffer,      0,retinalSize*retinalSize*sizeof(unsigned long));
    memset(timeBufferRight, 0,retinalSize*retinalSize*sizeof(unsigned long));

    printf("initialisation of unmask object correctly ended \n");

    return true;
}

unmask::~unmask() {

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

void unmask::resetTimestamps() {
    for (int i=0 ; i < retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
        timeBufferRight[i] = 0;
    }
    //verb = true;
    lasttimestamp = 0;
    lasttimestampright = 0;
}

void unmask::resetTimestampLeft() {
    for (int i=0 ; i < retinalSize * retinalSize; i++){
        timeBuffer[i] = 0;
    }
    lasttimestamp = 0;
}

void unmask::resetTimestampRight() {
    for (int i=0 ; i < retinalSize * retinalSize; i++){
        timeBufferRight[i] = 0;
    }
    lasttimestampright = 0;
}



unsigned long* unmask::getTimeBuffer(bool camera) {
    if (camera)
        return timeBuffer;
    else
        return timeBufferRight;
}

void unmask::run() {
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
    //cartX = retinalSize - cartX;

    // // // //if(cartX!=0) {
    // // // //    printf("retinalSize %d cartX %d cartY %d camera %d \n",retinalSize,cartX, cartY,camera);
    // // // //}
    // // // //printf("lastTimeStamp %08X \n", lasttimestamp);

    // camera is unmasked as left 0, right -1. It is converted in left 1, right 0
    // camera = camera + 1;
    // // // //printf("Camera %d polarity %d  \n", camera, polarity);
    // now camera: LEFT 1, RIGHT 0

    if(camera == 0) {  // ---- camera left -------------------
        if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
            validLeft =  true;
        }

        //TODO:: remove verb if not necessary
        //if(verb) {
            //for (int i = 0; i < 2; i ++) {
            //printf("verb is true %llu %llu \n", timestamp, lasttimestamp);
            //}
            //lasttimestamp = 0;
            //resetTimestamps();
        //}


        if(timestamp > lasttimestamp) {
            lasttimestamp = timestamp;
        }

        if(timeBuffer[cartX + cartY * retinalSize] < timestamp) {
            if(polarity == 0) {
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

        //TODO : remove if not necessary
        //if(verb) {
            //for (int i = 0; i < 2; i ++) {
            //printf("%llu \n", lasttimestamp);
            //}
            //lasttimestampright = 0;
            //resetTimestamps();
        //}

        if( timestamp > lasttimestampright){
            lasttimestampright = timestamp;
        }

        if (timeBufferRight[cartX + cartY * retinalSize] < timestamp) {
            if(polarity == 0) {
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


void unmask::addCLE(eEvent* qevt) {
    if(qevt->getType()=="CLE") {
        //ClusterEvent* ptr=dynamic_cast<ClusterEvent*>(qevt);
        //ptr->getXCoc();
        //ptr->getYCoc();
    }
    else if(qevt->getType()=="CLE-F0") {
        //ClusterEventFeatures1* ptr=dynamic_cast<ClusterEventFeatures1*>(qevt);
    }
    else if(qevt->getType()=="CLE-F1") {
        //ClusterEventFeature1* ptr=dynamic_cast<ClusterEventFeature1*>(qevt);
    }

    else if(qevt->getType()=="CLE-F2") {
        ClusterEventFeatures2* ptr=dynamic_cast<ClusterEventFeatures2*>(qevt);
        mutexCLELeft.wait();
        printf("received CLE_F2 from %d \n",ptr->getChannel() );
        if(ptr->getChannel() == 0) {

            _bufferCLELeft->xCog  = ptr->getYCog();
            _bufferCLELeft->yCog  = 128 - ptr->getXCog();
            _bufferCLELeft->xSize = ptr->getXSize();
            _bufferCLELeft->ySize = ptr->getYSize();
            _bufferCLELeft->numAE = ptr->getNumAE();
            _bufferCLELeft->id    = ptr->getId();
            if (countCLE < 10) {
                _bufferCLELeft++;
                countCLE++;
            }
        }
        mutexCLELeft.post();

        if(ptr->getChannel() == 1) {

            _bufferCLERight->xCog  = ptr->getYCog();
            _bufferCLERight->yCog  = 128 - ptr->getXCog();
            _bufferCLERight->xSize = ptr->getXSize();
            _bufferCLERight->ySize = ptr->getYSize();
            _bufferCLERight->numAE = ptr->getNumAE();
            _bufferCLERight->id     = ptr->getId();
            if (countCLERight < 10) {
                _bufferCLERight++;
                countCLERight++;
            }

        }

    }

    else if(qevt->getType()=="CLE-F3") {
        //ClusterEventFeature3* ptr=dynamic_cast<ClusterEventFeature3*>(qevt);
    }


}

void unmask::addHGE(eEvent* e) {
    HoughEvent* ptr=dynamic_cast<HoughEvent*>(e);
    //reprHGE _hge;

    if(!ptr->getChannel()) {
        //printf("adding HGE left \n");
        //left camera
        mutexHGELeft.wait();
        _bufferHGELeft->x      = ptr->getXCoc();
        _bufferHGELeft->y      = ptr->getYCoc();
        _bufferHGELeft->radius = ptr->getRadius();
        _bufferHGELeft->timestamp = timestamp;

        _bufferHGELeft++;
        mutexHGELeft.post();
    }
    else {
        // right camera
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
    unsigned long timestamp;

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
            //chksum = packets->get(i).asInt() % 255;
            //str[i] = (char) chksum;
        }
        //printf("%s \n", packets->toString().c_str());
        //fprintf(uEvents,"chksum: %s \n", str.c_str());
        fprintf(uEvents,"--- \n");
#endif
        //packets->pop(); // pop out necessary `till the error in reception found
        //packets->pop(); // pop out necessary `till the error in reception found
        //printf("size after pop %d \n", packets->size());

        //-- decoding the packet -------
        if(eEvent::decode(*packets,q)) {
            //printf("pointer %08X \n",  &q);
            //printf("deque size %d \n \n", (int) q.size());
            int dequeSize = q.size();

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

                        //identified an time stamp event
                        lastRecTimestamp = (unsigned int) ptr->getStamp();
                        //printf("lastTimestamp Received %08x \n", lastRecTimestamp);
                    }
                    else if(q[evt]->getType()=="CLE") {
                        //printf("timestamp \n");
                        ClusterEvent* ptr=dynamic_cast<ClusterEvent*>(q[evt]);

                        // code for CLE
                        //printf("received CLUSTER EVENT type: %s \n", ptr->getType());
                        //ptr->getXCog();
                        //ptr->getYCog();
                        //addCLE(q[evt]);
                    }
                    else if(q[evt]->getType()=="CLE-F0") {
                        printf("CLE_F0  \n");
                        printf("CLE_F0  \n");
                        printf("CLE_F0  \n");

                        ClusterEvent* ptr=dynamic_cast<ClusterEvent*>(q[evt]);

                        // code for CLE
                        //printf("received CLUSTER EVENT type: %s \n", ptr->getType());
                        //ptr->getXCog();
                        //ptr->getYCog();

                    }
                    else if(q[evt]->getType()=="CLE-F1") {
                        printf("CLE_F1  \n");
                        printf("CLE_F1  \n");
                        printf("CLE_F1  \n");

                        ClusterEvent* ptr=dynamic_cast<ClusterEvent*>(q[evt]);

                        // code for CLE
                        //printf("received CLUSTER EVENT type: %s \n", ptr->getType());
                        //ptr->getXCog();
                        //ptr->getYCog();

                    }
                    else if(q[evt]->getType()=="CLE-F2") {
                        printf("CLE_F2  \n");
                        printf("CLE_F2  \n");
                        printf("CLE_F2  \n");

                        ClusterEvent* ptr=dynamic_cast<ClusterEvent*>(q[evt]);

                        // code for CLE
                        //printf("received CLUSTER EVENT type: %s \n", ptr->getType());
                        //ptr->getXCog();
                        //ptr->getYCog();
                        int channel = ptr->getChannel();
                        printf("channed of the CLUSTER EVENT %d \n", channel);

                        addCLE(q[evt]);
                    }
                    else if(q[evt]->getType()=="HGE") {

                        HoughEvent* ptr=dynamic_cast<HoughEvent*>(q[evt]);

                        //code for HGE
                        //printf("received HOUGH EVENT type: %s \n", ptr->getType());

                        addHGE(q[evt]);
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

            //printf("ERROR in DECODING  \n");
        }


    } // end else packets->isNull;
}



void unmask::unmaskData(char* i_buffer, int i_sz, bool verb) {
    //AER_struct sAER
    count++;
    bool endofwords = false;
    int num_events;
    num_events = i_sz / 4;
    //cout << "Number of the received events : " << num_events<< endl;

    //create a pointer that points every 4 bytes
    u32* buf2 = (u32*)i_buffer;
    u16* buf1 = (u16*)i_buffer;
    unsigned long timestamp;


    //for (int i = 0 ; i < i_sz ; i+=4) {
    //    unsigned int part_1 = 0xFF & i_buffer[i];    //extracting the 1 byte
    //    unsigned int part_2 = 0xFF & i_buffer[i+1];  //extracting the 2 byte
    //    unsigned int part_3 = 0xFF & i_buffer[i+2];  //extracting the 3 byte
    //    unsigned int part_4 = 0xFF & i_buffer[i+3];  //extracting the 4 byte
    //    //float blob = (part_1)|(part_2<<8);
    //    blob      = (part_1)|(part_2<<8);          //16bits
    //    //float timestamp = ((part_3)|(part_4<<8));
    //    timestamp = ((part_3)|(part_4<<8));        //16bits
    //    printf(">>>>>>>>> %08X %08X \n",blob,timestamp);
        //if(i == 100){
            //printf("Saving in file \n");
            //printf("---> %08X %08X \n",blob,timestamp);
            //fwrite(&sz, sizeof(int), 1, raw);
            //fwrite(buffer, 1, sz, raw);
        //}
    //}

    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        if(!dvsMode) {
            // unmask the data ( first 4 byte blob, second 4 bytes timestamp)
            //unsigned long blob      = buf2[2 * evt];
            //unsigned long t         = buf2[2 * evt + 1];

            // unmask the data ( first 4 byte ts, second 4 bytes blob)
            //unsigned long t      = buf2[2 * evt];
            //unsigned long blob   = buf2[2 * evt + 1];
            unsigned long v = buf2[evt];
            //buf2++;
            unsigned long t,blob;
            unsigned long v_unmasked = (v & 0xFC000000) >> 26;

            if(v_unmasked == 0x22) {
                printf("Wrap Around!!!! \n");
                /*
                if(!wrapOcc) {
                    printf("LasTimestamp \n");
                    //lasttimestamp = 0;
                    //lasttimestampright = 0;
                    wrapOcc = true;
                    resetTimestampLeft();
                    resetTimestampRight();
                }
                //buf2[2 * evt] = 0;       // removing the TS_WA from the buffer
                continue;
                */
                resetTimestampLeft();
                resetTimestampRight();

            }
            else if(v_unmasked == 0x00) {
                //if (v!=0) {
                //    printf("%08X %08X blob \n",timestamp,v);
                //}
                //blob
                blob = v;
                blob &= 0xFFFF;
                unmaskEvent( (unsigned int) blob, cartX, cartY, polarity, camera);
                //printf("%08X %d \n", blob, camera);
                endofwords = true;
            }
            else if(v_unmasked == 0x20) {
                //timestamp
                //if (v!=0) {
                //    printf("%08X timestamp \n",v);
                //}
                t = v;
                t &= 0x00FFFFFF;
                timestamp =  (unsigned long) t;
                if(timestamp > lasttimestamp) {
                    lasttimestamp = timestamp;
                }
                if( timestamp > lasttimestampright) {
                    lasttimestampright = timestamp;
                }
            }
            else {
                printf("NOT Recognized!!! \n");
            }
        }

        if(endofwords) {
            endofwords = false;
            // filling the image buffer after the unmasking
            //checking outoflimits in dimension
            //correcting the orientation of the camera
            cartY = retinalSize - cartY - 1;   //corrected the output of the camera (flipped the image along y axis)
            //cartX = retinalSize - cartX;

            //if(cartX!=0)
            //    printf("retinalSize %d cartX %d cartY %d camera %d \n",retinalSize,cartX, cartY,camera);
            //printf("lastTimeStamp %08X \n", lasttimestamp);

            //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
            //camera = camera + 1;

            //printf("Camera %d polarity %d  \n", camera, polarity);
            //camera: LEFT 1, RIGHT 0

            if(!camera) {
                if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
                    validLeft =  true;
                }

                if(timeBuffer[cartX + cartY * retinalSize] < timestamp) {
                    if(polarity > 0) {
                        buffer    [cartX + cartY * retinalSize] += responseGradient;
                        timeBuffer[cartX + cartY * retinalSize]  = timestamp;

                        if( buffer[cartX + cartY * retinalSize] > 127) {
                            buffer[cartX + cartY * retinalSize] = 127;
                        }
                    }
                    else if(polarity < 0) {
                        buffer    [cartX + cartY * retinalSize] -= responseGradient;
                        timeBuffer[cartX + cartY * retinalSize]  = timestamp;

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

                if (timeBufferRight[cartX + cartY * retinalSize] < timestamp) {
                    if(polarity > 0) {
                        bufferRight    [cartX + cartY * retinalSize] += responseGradient;
                        timeBufferRight[cartX + cartY * retinalSize]  = timestamp;

                        if( bufferRight[cartX + cartY * retinalSize] > 127) {
                            bufferRight[cartX + cartY * retinalSize] = 127;
                        }
                    }
                    else if(polarity < 0) {
                        bufferRight[cartX + cartY * retinalSize] -= responseGradient;
                        timeBufferRight[cartX + cartY * retinalSize] = timestamp;

                        if (bufferRight[cartX + cartY * retinalSize] < -127) {
                            bufferRight[cartX + cartY * retinalSize] = -127;
                        }
                    }
                }
            } //end camera
        } //endofwords
    } //end of for
}



/* backup
void unmask::unmaskData(char* i_buffer, int i_sz, bool verb) {
    //AER_struct sAER
    count++;
    if(verb)
        printf("Unmasking with the timestamp %lu \n", lasttimestamp);

    int num_events;
    if(dvsMode) {
        num_events = i_sz / 4;    // pointing to events made of 4 bytes
    }
    else {
        num_events = i_sz / 8;    // pointing to events made of 8 bytes = 64 bits
    }
    //cout << "Number of the received events : " << num_events<< endl;

    //create a pointer that points every 4 bytes and 2 bytes (DVS mode)
    uint32_t* buf2 = (uint32_t*)i_buffer;
    uint16_t* buf1 = (uint16_t*)i_buffer;
    unsigned long timestamp;


    //for (int i = 0 ; i < i_sz ; i+=4) {
    //    unsigned int part_1 = 0xFF & i_buffer[i];    //extracting the 1 byte
    //    unsigned int part_2 = 0xFF & i_buffer[i+1];  //extracting the 2 byte
    //    unsigned int part_3 = 0xFF & i_buffer[i+2];  //extracting the 3 byte
    //    unsigned int part_4 = 0xFF & i_buffer[i+3];  //extracting the 4 byte
    //    //float blob = (part_1)|(part_2<<8);
    //    blob      = (part_1)|(part_2<<8);          //16bits
    //    //float timestamp = ((part_3)|(part_4<<8));
    //    timestamp = ((part_3)|(part_4<<8));        //16bits
    //    printf(">>>>>>>>> %08X %08X \n",blob,timestamp);
        //if(i == 100){
            //printf("Saving in file \n");
            //printf("---> %08X %08X \n",blob,timestamp);
            //fwrite(&sz, sizeof(int), 1, raw);
            //fwrite(buffer, 1, sz, raw);
        //}
    //}



    //eldesttimestamp = 0;
    int i = 0;
    for (int evt = 0; evt < num_events; evt++) {
        if(!dvsMode) {
            // unmask the data ( first 4 byte blob, second 4 bytes timestamp)
            //unsigned long blob      = buf2[2 * evt];
            //unsigned long t         = buf2[2 * evt + 1];

            // unmask the data ( first 4 byte ts, second 4 bytes blob)
            //unsigned long t      = buf2[2 * evt];
            //unsigned long blob   = buf2[2 * evt + 1];
            unsigned long t = *buf2;
            buf2++;
            unsigned long blob;

            if(t == 0x88000000) {
                printf("Wrap Around!!!! \n");
                continue;

            } else {
                blob = *buf2;
                buf2++;
            }



            //
            //if (t > 0x00000100) {
            //    wrapOcc = false;
            //}


            //printf("0x%x 0x%x \n",blob, timestamp);

            // here we zero the higher two bytes of the address!!! Only lower 16bits used!
            blob &= 0xFFFF;
            t &= 0x00FFFFFF;
            //unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
            unmaskEvent( (unsigned int) blob, cartX, cartY, polarity, camera);
            timestamp =  (unsigned long) t;



            //if((blob!=0) && (t!=0))
            //    printf(">>>>>>>>> %08X %08X \n",blob,t);
        }
        else {  //in dvsMode
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

        i += 4; // increment the counter for DVS of 4bytes


        // filling the image buffer after the unmasking

        //checking outoflimits in dimension

        //correcting the orientation of the camera
        cartY = retinalSize - cartY - 1;   //corrected the output of the camera (flipped the image along y axis)

        //assert(cartX < retinalSize);
        //assert(cartX >= 0 );
        //assert(cartY < retinalSize);
        //assert(cartY >= 0);
        if((cartX < 0)||(cartX > retinalSize)){
            cartX = 0;
        }
        if((cartY < 0)||(cartY> retinalSize)){
            cartY = 0;
        }


        //cartX = retinalSize - cartX;

        //if(cartX!=0)
        //    printf("retinalSize %d cartX %d cartY %d camera %d \n",retinalSize,cartX, cartY,camera);
        //printf("lastTimeStamp %08X \n", lasttimestamp);

        //camera is unmasked as left 0, right -1. It is converted in left 1, right 0
        camera = camera + 1;
        //printf("Camera %d polarity %d  \n", camera, polarity);
        //camera: LEFT 1, RIGHT 0

        if(camera) {
            if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
                validLeft =  true;
            }


            //TODO:: remove verb if not necessary
            if(verb) {
                //for (int i = 0; i < 2; i ++) {
                //printf("verb is true %llu %llu \n", timestamp, lasttimestamp);
                    //}
                //lasttimestamp = 0;
                //resetTimestamps();
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

                    if(asvMode){
                        if(!((cartX>=7)&&(cartX<16)&&(cartY>=7)&&(cartY<16))){
                            buffer[cartX + 1 + cartY * retinalSize]       = responseGradient;
                            timeBuffer[cartX + 1  + cartY * retinalSize]  = timestamp;
                            buffer[cartX + (cartY + 1)  * retinalSize]    = responseGradient;
                            timeBuffer[cartX + (cartY + 1) * retinalSize] = timestamp;
                            buffer[cartX + 1 + (cartY + 1) * retinalSize]       = responseGradient;
                            timeBuffer[cartX + 1  + (cartY + 1) * retinalSize]  = timestamp;
                        }
                    }

                }
                else if(polarity < 0) {
                    buffer[cartX + cartY * retinalSize] -= responseGradient;
                    timeBuffer[cartX + cartY * retinalSize] = timestamp;

                    if (buffer[cartX + cartY * retinalSize] < -127) {
                        buffer[cartX + cartY * retinalSize] = -127;
                    }

                    if(asvMode) {
                        if(!((cartX>=7)&&(cartX<16)&&(cartY>=7)&&(cartY<16))){
                            buffer[cartX + 1 + cartY * retinalSize]       = -responseGradient;
                            timeBuffer[cartX + 1  + cartY * retinalSize]  = timestamp;
                            buffer[cartX + (cartY + 1)  * retinalSize]    = -responseGradient;
                            timeBuffer[cartX + (cartY + 1) * retinalSize] = timestamp;
                            buffer[cartX + 1 + (cartY + 1) * retinalSize]       = -responseGradient;
                            timeBuffer[cartX + 1  + (cartY + 1) * retinalSize]  = timestamp;
                        }
                    }
                }
            }
        }
        else { // --------  camera right ------------------------

            if((cartX!=0) &&( cartY!=0) && (timestamp!=0)) {
                validRight =  true;
            }

            //TODO : remove if not necessary
            if(verb) {
                //for (int i = 0; i < 2; i ++) {
                //printf("%llu \n", lasttimestamp);
                    //}
                //lasttimestampright = 0;
                //resetTimestamps();
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
                else if(polarity < 0) {
                    bufferRight[cartX + cartY * retinalSize] -= responseGradient;
                    timeBufferRight[cartX + cartY * retinalSize] = timestamp;

                    if (bufferRight[cartX + cartY * retinalSize] < -127) {
                        bufferRight[cartX + cartY * retinalSize] = -127;
                    }
                }
            }
        } //end camera
    } //end of for
}

*/


/*void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol, short& camera) {
    y =       (short) (retinalSize - 1) - (short)((evPU & xmask) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    x =       (short) ((evPU & ymask)     >> yshift);
    pol =    ((short) ((evPU & polmask)   >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    camera = ((short) ( evPU & cameramask) >> camerashift);	        //0 LEFT, 1 RIGHT
    }*/

void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol, short& camera) {
    y =       (short)((evPU & xmaskshort) >> xshift);
    //y = (short) ((evPU & xmask)>>xshift);
    x =       (short) ((evPU & ymaskshort)       >> yshift);
    pol =    ((short) ((evPU & polmaskshort)     >> polshift)==0)?1:-1;	  //+1 ON, -1 OFF
    camera =  (short) (( evPU & cameramaskshort) >> camerashift);	      //0 LEFT, 1 RIGHT
    //printf("1evPU%x polmask%x polshift%x \n");
}

void unmask::unmaskEvent(unsigned int evPU, short& x, short& y, short& pol) {
    y =       (short) (retinalSize - 1) - (short)((evPU & xmaskshort) >> xshift);
    x =       (short) ((evPU & ymaskshort)      >> yshift);
    pol =    ((short) ((evPU & polmaskshort)    >> polshift)==0)?-1:1;	//+1 ON, -1 OFF
    //printf("2evPU%x polmask%x polshift%x \n",evPU, polmask,polshift);
}

/* function used fro AEX extracting pixelcoordinates,  polarity and camera*/
void unmask::unmaskEvent(long int evPU, short& x, short& y, short& pol, short& camera) {
    //x =      (short) (retinalSize - 1) - (short)((evPU & xmask) >> xshift);
    x =      (short) ((evPU & xmask)      >> xshift);
    y =      (short) ((evPU & ymask)      >> yshift);
    pol =    (short) ((evPU & polmask)    >> polshift)==0?-1:1;      //+1 ON, -1 OFF
    //printf("3evPU%x polmask%x polshift%x \n");
    camera = (short) ((evPU & cameramask) >> camerashift);	         //0 LEFT, 1 RIGHT
}

void unmask::threadRelease() {
    delete[] buffer;
    delete[] timeBuffer;
    delete[] bufferRight;
    delete[] timeBufferRight;
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


//----- end-of-file --- ( next line intentionally left blank ) ------------------

