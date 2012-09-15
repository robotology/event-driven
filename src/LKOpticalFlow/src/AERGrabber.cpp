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


#include <iCub/AERGrabber.h>


AERGrabber::AERGrabber(yarp::os::Semaphore * sigEvents, unsigned long t)
//            : yarp::os::BufferedPort<yarp::os::Bottle>() , eBufferMutex(1){
            : yarp::os::BufferedPort<eventBuffer>() , eBufferMutex(1){

	evntsMutex = sigEvents;
	frameInv = t;
	lastFrameTS = 0;
    wrapAddup = 0;
    onTimestamps.resize(RETINA_SIZE_R + 2*RELIABLE_NGHBRHD, RETINA_SIZE_C + 2*RELIABLE_NGHBRHD);
    onTimestamps.initialize(0);
    offTimestamps.resize(RETINA_SIZE_R + 2*RELIABLE_NGHBRHD, RETINA_SIZE_C + 2*RELIABLE_NGHBRHD);
    offTimestamps.initialize(0);

    //revtBuffer.reserve(100);
    //evtFrame.reserve(2000);

}

void AERGrabber::onRead(eventBuffer & eBuffer){
//void AERGrabber::onRead(yarp::os::Bottle & inBottle){
    int eventNo;
    short cartX, cartY, polarity, camera;
    short evntRowIdx, evntClmnIdx;
    unsigned long tsPacket, timestamp, wraparounTS;
    unsigned long blob;


//readJEAR(eBuffer);
//return;

//    const char * bottleBuff = inBottle.toBinary();
//    eventNo = inBottle.size();
//    u32* buf2 = (u32*) bottleBuff;

    eventNo = eBuffer.get_sizeOfPacket()/4;
    u32* buf2 = (u32*) eBuffer.get_packet();

//    cout << eventNo << endl;
//    if (evtBuffer.size() > 200){
//        cout << evtBuffer.size() << " " << eventNo << endl;
//        return;
//    }

 //   cout << "eventNo" << eventNo << endl;

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

        if (camera == -1 || blob == 0 /*|| polarity==-1*/) // 0, to work with Right camera -- -1, to work with the left one
          continue;// not consider the other camera

        //printf("%08X %08X\n",blob,tsPacket);


        //if it's a real event create CameraEvent and put it in the buffer using semaphore eBufferMutex
        if (isReliableEvent(cartY, cartX, polarity, timestamp)){

            evntClmnIdx = cartX  + SPATIAL_MARGINE_ADDUP;
            evntRowIdx = cartY  + SPATIAL_MARGINE_ADDUP;

//            if (lastIdx < BUFFER_SIZE){
                CameraEvent * evnt = new CameraEvent(evntRowIdx, evntClmnIdx, polarity, timestamp);
                evtFrame.push_back(evnt);
//            }

        }

        if (timestamp - lastFrameTS > frameInv){ // when timeInv(nano secons) passes, send a signal
        
        	lastFrameTS= timestamp;

            if (evtFrame.size() > 0){
        	   eBufferMutex.wait();
        	   evtBuffer.push(evtFrame);
        	   eBufferMutex.post();
        	   evntsMutex ->post();
           	   evtFrame.clear();        	   
            }
//            else
//                freeBuffer(evtFrame);
        }

    }

}


void AERGrabber::readJEAR(eventBuffer & eBuffer){
    int eventNo;
	short cartX, cartY, polarity, camera;
	short evntRowIdx, evntClmnIdx;
	unsigned long tsPacket, timestamp, wraparounTS;
	unsigned long blob;


   	if (evtBuffer.size() > 100){
   	    cout << evtBuffer.size() <<endl;
	    return;
   	}

    eventNo = eBuffer.get_sizeOfPacket()/4;
	u32* buf2 = (u32*) eBuffer.get_packet();

   	for (int evt = 0; evt < eventNo; ) {
		// unmask the data ( first 4 bytes timestamp, second 4 bytes address)
		blob = buf2[evt++];
		timestamp = buf2[evt++];
		blob &= 0xFFFF; // here we zero the higher two bytes of the address!!! Only lower 16bits used!

		unmasker.unmaskEvent((unsigned int) blob, cartY, cartX, polarity, camera);
	    cartY = 127 - cartY;
	   // cartX = 127 - cartX;

		//printf("%08X %08X\n",blob,tsPacket);

		if (blob == 0 /*|| polarity==-1*/) // 0, to work with Right camera -- -1, to work with the left one
		  continue;// not consider the other camera


		//if it's a real event create CameraEvent and put it in the buffer using semaphore eBufferMutex
		if (isReliableEvent(cartY, cartX, polarity, timestamp)){
			evntClmnIdx = cartX  + SPATIAL_MARGINE_ADDUP;
			evntRowIdx = cartY  + SPATIAL_MARGINE_ADDUP;
//            if (lastIdx < BUFFER_SIZE){
				CameraEvent * evnt = new CameraEvent(evntRowIdx, evntClmnIdx, polarity, timestamp);
				evtFrame.push_back(evnt);
//            }
		}

//		cout << cartX << " "  << cartY << " "  << polarity << " " << timestamp << endl;

		if (timestamp - lastFrameTS > frameInv){ // when timeInv(nano secons) passes, send a signal
			lastFrameTS= timestamp;
			if (evtFrame.size() > 0){
			   eBufferMutex.wait();
			   evtBuffer.push(evtFrame);
			   eBufferMutex.post();
			   evntsMutex ->post();
			   evtFrame.clear();
			}
//            else
//                freeBuffer(evtFrame);
		}

	}


}

bool AERGrabber::isReliableEvent (short row, short clmn, short polarity, unsigned long ts){
    TIMESTAMP_TYPE tmp;


    if (polarity == 1){
        tmp = onTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD);

        //update the pixel neighborhood in timestamps Matrix with the new value of ts
 //       onTimestamps.updateSubMatrix(row, clmn, ts,              // row + RELIABLE_NGHBRHD - RELIABLE_NGHBRHD = row
 //                                  2*RELIABLE_NGHBRHD + 1, 2*RELIABLE_NGHBRHD+1);

       
        onTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD) = ts;

    }
    else{
        tmp = offTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD);

        //update the pixel neighborhood in timestamps Matrix with the new value of ts
//        offTimestamps.updateSubMatrix(row, clmn, ts,              // row + RELIABLE_NGHBRHD - RELIABLE_NGHBRHD = row
//                                   2*RELIABLE_NGHBRHD + 1, 2*RELIABLE_NGHBRHD+1);

        offTimestamps(row + RELIABLE_NGHBRHD, clmn + RELIABLE_NGHBRHD) = ts;

    }


    if (ts - tmp > RELIABLE_EVENT_THRSHLD /*&& ts - tmp < 2000*/ )
        return false;

    return true;
}

CameraEvent ** AERGrabber::getEvents(int & evenNo){
    CameraEvent ** result = NULL;
    CameraEvent ** ptr;
    evenNo = 0;

    vector< CameraEvent * > eFrame;

    eBufferMutex.wait();
    if (evtBuffer.size()){
        eFrame = evtBuffer.front();
        evtBuffer.pop();
    }
    eBufferMutex.post();

    evenNo = eFrame.size();

    if (evenNo > 0){
        result = new CameraEvent* [evenNo];   //allocate memory for the result
        ptr = result;
        //copy from evntBuffer[firstIdx-lastIdx]
        for (int cnt = 0; cnt < evenNo; ++cnt) {
            *ptr++ = eFrame.at(cnt);
        }
    }

    return result;
}

inline void AERGrabber::freeBuffer(vector< CameraEvent * > & f){

    int sz = f.size();
    for (int i = 0; i < sz; ++i) {
        delete f.at(i);
    }
    f.clear();

}

AERGrabber::~AERGrabber(){
	cout << "AER grabber start cleaning" << endl;
	int sz = evtBuffer.size();

	for (int j = 0; j < sz; ++j) {
	    freeBuffer( evtBuffer.front());
	    evtBuffer.pop();
    }

    cout << "AER grabber is closed happily" << endl;
}

