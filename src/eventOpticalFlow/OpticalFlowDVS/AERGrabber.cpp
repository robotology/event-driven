/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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


#include "AERGrabber.h"


AERGrabber::AERGrabber(yarp::os::Semaphore * sigEvents, unsigned long t)
            : yarp::os::BufferedPort<eventBuffer>() , eBufferMutex(1){

	evntsMutex = sigEvents;
	frameInv = t;
    firstIdx = 0;
    lastIdx = 0;
    lastFrameTS = 0;
    onTimestamps.resize(RETINA_SIZE_R + 2*RELIABLE_NGHBRHD, RETINA_SIZE_C + 2*RELIABLE_NGHBRHD);
    onTimestamps.initialize(0);
    offTimestamps.resize(RETINA_SIZE_R + 2*RELIABLE_NGHBRHD, RETINA_SIZE_C + 2*RELIABLE_NGHBRHD);
    offTimestamps.initialize(0);
}


void AERGrabber::onRead(eventBuffer & eBuffer){
    unmask unmasker;
    int eventNo;
    short cartX, cartY, polarity, camera;
    short evntRowIdx, evntClmnIdx;
    unsigned long tsPacket, timestamp, wraparounTS;
    unsigned long blob;


    eventNo = eBuffer.get_sizeOfPacket()/4;

    uint32_t* buf2 = (uint32_t*) eBuffer.get_packet();
    for (int evt = 0; evt < eventNo; ) {
        // unmask the data ( first 4 bytes timestamp, second 4 bytes address)
        tsPacket = buf2[evt++];
        //Check if s tamistamp wrap around occured
        if ((tsPacket & 0xFC000000) == 0x88000000){ // if it's TSWA then skip it
            wrapAddup += TS_RANGE;
            tsPacket = buf2[evt++];
        }
        timestamp = (tsPacket &  0x03FFFFFF) + wrapAddup;
        blob = buf2[evt++];

        blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!
        unmasker.unmaskEvent((unsigned int) blob, cartX, cartY, polarity, camera);
       // cartY = 127 - cartY;
       // cartX = 127 - cartX;

        if (camera == 0 || blob == 0 /*|| polarity==-1*/) // 0, to work with Right camera -- -1, to work with the left one
          continue;// not consider the other camera

        //printf("%08X %08X\n",blob,tsPacket);


        //if it's a real event create CameraEvent and put it in the buffer using semaphore eBufferMutex
        if (isReliableEvent(cartY, cartX, polarity, timestamp)){

            evntClmnIdx = cartX  + SPATIAL_MARGINE_ADDUP;
            evntRowIdx = cartY  + SPATIAL_MARGINE_ADDUP;

            eBufferMutex.wait();
            if (lastIdx < BUFFER_SIZE){
                CameraEvent * evnt = new CameraEvent(evntRowIdx, evntClmnIdx, polarity, timestamp);
                evntBuffer [lastIdx] = evnt;
                lastIdx++;
            }
            eBufferMutex.post();
        }

        if (timestamp - lastFrameTS > frameInv){ // when timeInv(nano secons) passes, send a signal
        	evntsMutex ->post();
        	lastFrameTS= timestamp;
        }
    }
}

bool AERGrabber::isReliableEvent (short row, short clmn, short polarity, unsigned long ts){
    TIMESTAMP_TYPE tmp;


    if (polarity == 1){
        tmp = onTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD);

        //update the pixel neighborhood in timestamps Matrix with the new value of ts
        onTimestamps.updateSubMatrix(row, clmn, ts,              // row + RELIABLE_NGHBRHD - RELIABLE_NGHBRHD = row
                                   2*RELIABLE_NGHBRHD + 1, 2*RELIABLE_NGHBRHD+1);

    }
    else{
        tmp = offTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD);

        //update the pixel neighborhood in timestamps Matrix with the new value of ts
        offTimestamps.updateSubMatrix(row, clmn, ts,              // row + RELIABLE_NGHBRHD - RELIABLE_NGHBRHD = row
                                   2*RELIABLE_NGHBRHD + 1, 2*RELIABLE_NGHBRHD+1);

    }


    if (ts - tmp > RELIABLE_EVENT_THRSHLD)
        return false;

    return true;
}

CameraEvent ** AERGrabber::getEvents(int & evenNo){
    CameraEvent ** result = NULL;
    CameraEvent ** ptr;
    evenNo = 0;

    eBufferMutex.wait();
    if (firstIdx < lastIdx ){
        evenNo = lastIdx - firstIdx;
        result = new CameraEvent* [lastIdx - firstIdx];   //allocate memory for the result
        ptr = result;
        //copy from evntBuffer[firstIdx-lastIdx]
        for (int cnt = firstIdx; cnt < lastIdx; ++cnt) {
            *ptr++ = evntBuffer[cnt];
        }
        firstIdx = 0;
        lastIdx =  0;
    }
    eBufferMutex.post();
    return result;
}


AERGrabber::~AERGrabber(){
	cout << "AER grabber start cleaning" << endl;
    for (int i = 0; i < lastIdx; ++i) {
        delete evntBuffer[i];
    }

    cout << "AER grabber is closed happily" << endl;
}

