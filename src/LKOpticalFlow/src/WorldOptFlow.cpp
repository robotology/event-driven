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


#include <iCub/WorldOptFlow.h>

WorldOptFlow::WorldOptFlow(AERGrabber * inPortPtr, BufferedPort<VelocityBuffer> * outFlowPort,
                     unsigned long sampleInv,
                     MyMatrix<POLARITY_TYPE> * wStatus, MyMatrix<POLARITY_TYPE> * pWStatus,
                     MyMatrix<TIMESTAMP_TYPE> * ts, yarp::os::Semaphore * eventsSignal)
                       : RateThread(1), localFlw(LUCAS_KANADE_NGHBR) {
    inPort = inPortPtr;
    worldStatus = wStatus;
    prevWorldStatus = pWStatus;
    timestamps = ts;
    localFlw.setGuaWeights(LUCAS_KANADE_GUAS_STDDEV);
    outPort = outFlowPort;
    newEventsSignal = eventsSignal;

    eventBuffersSize = SPDerivative_WNDW_SZ + 1;
    bufferInitialized = 0;
    sampleTime = sampleInv;



}

void WorldOptFlow::initialize(int step){
    int eventNo;
    CameraEvent ** eventsBfr;
    CameraEvent * evntPtr;
    short rwIdx, clmnIdx;


    eventsBfr = inPort-> getEvents(eventNo);
    eventBuffers[step] = eventsBfr;
    eventNosBuffer[step] = eventNo;

    for (int i = 0; i < eventNo; ++i) {
        evntPtr = eventsBfr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus->operator ()(rwIdx, clmnIdx) += POLARITY_WEIGHT*evntPtr->getPolarity();
    }

}

void WorldOptFlow::cleanup(){
	CameraEvent ** eventsBfr;

	cout << "WorldFlow start cleaning" << endl;

	for (int j = 0; j < eventBuffersSize - 1; ++j) {
		eventsBfr = eventBuffers[j];
		if (eventsBfr != NULL){
		   // each element of array is a CameraEvent *
		   for (int i = 0; i < eventNosBuffer[j]; ++i) {
 			   delete *(eventsBfr + i);
		   }
		   delete [] eventsBfr;
	   }
	}


	delete [] eventBuffers;
	delete [] eventNosBuffer;
	cout << "world flow is cleaned up" << endl;
}


bool WorldOptFlow::threadInit(){

	eventBuffers = new CameraEvent**[eventBuffersSize];
	eventNosBuffer = new int [eventBuffersSize];
	for (int i = 0; i < eventBuffersSize; ++i) {
		eventBuffers[i] = NULL;
		eventNosBuffer[i] = 0;
	}

	return true;
}


void WorldOptFlow::threadRelease(){

	cout << "calling thread release ... " << endl;

	cleanup();

	cout << "end thread release" << endl;
}


void WorldOptFlow::run(){
    static double last_read = 0;
    int eventNo;
    CameraEvent ** eventsBfr;
    CameraEvent * evntPtr;;
    int tmp;

    if (newEventsSignal->check()){
        last_read = yarp::os::Time::now();

        if (bufferInitialized < SPDerivative_WNDW_SZ){
            initialize(bufferInitialized);
            bufferInitialized++;
            return;
        }

        eventsBfr = inPort->getEvents(eventNo);
        *(eventBuffers + eventBuffersSize - 1) = eventsBfr; //eventBuffers[eventBuffersSize - 1] = eventsBfr;
        *(eventNosBuffer + eventBuffersSize - 1) = eventNo; //eventNosBuffer[eventBuffersSize - 1] = eventNo;

        updtWrldStus();

//      clock_t start= clock();

        calVelocities(eventsBfr, eventNo);

//        clock_t end = clock();
//        double elapsed = ( (double) (end - start) ) / CLOCKS_PER_SEC;
//        cout.precision(10);
//        cout << elapsed << " " << eventNo << endl;

        prevWorldStatus->updateMatrix(worldStatus);
//        updtPreWrldStus();

        //Release the memory for events
        eventsBfr = eventBuffers[0];
        if (eventsBfr != NULL){
           // each element of array is a CameraEvent *
           tmp = *eventNosBuffer/*eventNosBuffer[0]*/;
           for (int i = 0; i < tmp; ++i) {
               delete *(eventsBfr + i);
           }
           delete [] eventsBfr;
        }

        //Shift the eventBufffers one step
        for (int i = 1; i < eventBuffersSize; ++i) {
            *(eventBuffers + i-1) = *(eventBuffers + i); //eventBuffers[i-1] = eventBuffers[i];
            *(eventNosBuffer + i-1) = *(eventNosBuffer + i); // eventNosBuffer[i-1] = eventNosBuffer[i];
        }

    }
    else{
        double curr_time = yarp::os::Time::now();
        if (curr_time-last_read> .1){  //if it's a long time i have not received any thing then put some stuff on the
            *(eventBuffers  + eventBuffersSize - 1) = NULL;
            *(eventNosBuffer + eventBuffersSize - 1) = 0;
            updtWrldStus();
            updtPreWrldStus();

            //Release the memory for events
            eventsBfr = eventBuffers[0];
            if (eventsBfr != NULL){
               // each element of array is a CameraEvent *
               tmp = *eventNosBuffer/*eventNosBuffer[0]*/;
               for (int i = 0; i < tmp; ++i) {
                   delete *(eventsBfr + i);
               }
               delete [] eventsBfr;
            }

            //Shift the eventBufffers one step
            for (int i = 1; i < eventBuffersSize; ++i) {
                *(eventBuffers + i-1) = *(eventBuffers + i); //eventBuffers[i-1] = eventBuffers[i];
                *(eventNosBuffer + i-1) = *(eventNosBuffer + i); // eventNosBuffer[i-1] = eventNosBuffer[i];
            }
        }// end-if on time
    }
}


void WorldOptFlow::updtWrldStus(){
    CameraEvent * evntPtr;
    CameraEvent ** evntBffr;
    short rwIdx, clmnIdx;
    int tmp;

    evntBffr = *eventBuffers; //eventBuffers[0];
    tmp = *eventNosBuffer;
    for (int i = 0; i < tmp; ++i) {
        evntPtr = *(evntBffr + i ); //evntBffr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus->operator ()(rwIdx, clmnIdx) -= POLARITY_WEIGHT*evntPtr->getPolarity();
    }

    evntBffr = *(eventBuffers + eventBuffersSize - 1); eventBuffers[eventBuffersSize - 1];
    tmp = * (eventNosBuffer + eventBuffersSize -1);
    for (int i = 0; i < tmp ; ++i) {  //eventNosBuffer[eventBuffersSize -1]
        evntPtr = *(evntBffr + i); //evntBffr[i];
        rwIdx = evntPtr ->getRowIdx();
        clmnIdx = evntPtr -> getColumnIdx();
        worldStatus ->operator ()(rwIdx, clmnIdx) += POLARITY_WEIGHT * evntPtr->getPolarity();
    }

}


void WorldOptFlow::updtPreWrldStus(){
    CameraEvent * evntPtr;
      CameraEvent ** evntBffr;
      short rwIdx, clmnIdx;
      int tmp;

      evntBffr = *eventBuffers; //eventBuffers[0];
      tmp = *eventNosBuffer;
      for (int i = 0; i < tmp; ++i) {
          evntPtr = *(evntBffr + i ); //evntBffr[i];
          rwIdx = evntPtr ->getRowIdx();
          clmnIdx = evntPtr -> getColumnIdx();
          prevWorldStatus->operator ()(rwIdx, clmnIdx) -= POLARITY_WEIGHT*evntPtr->getPolarity();
      }

      evntBffr = *(eventBuffers + eventBuffersSize - 1); eventBuffers[eventBuffersSize - 1];
      tmp = * (eventNosBuffer + eventBuffersSize -1);
      for (int i = 0; i < tmp ; ++i) {  //eventNosBuffer[eventBuffersSize -1]
          evntPtr = *(evntBffr + i); //evntBffr[i];
          rwIdx = evntPtr ->getRowIdx();
          clmnIdx = evntPtr -> getColumnIdx();
          prevWorldStatus->operator ()(rwIdx, clmnIdx) += POLARITY_WEIGHT * evntPtr->getPolarity();
      }

}


void WorldOptFlow::calVelocities(CameraEvent ** evntBffr, int bffrSize){

	CameraEvent * evntPtr;
	short evtRw, evtClm;
	VelocityBuffer vlctyBuffer(sampleTime);
	double velocity [3];


	for (int cntr = 0; cntr < bffrSize; ++cntr) {

		*velocity = 0;
		*(velocity + 1) = 0;

		evntPtr = evntBffr[cntr];

		if (evntPtr == 0)
			continue;


		evtRw = evntPtr->getRowIdx();
		evtClm = evntPtr->getColumnIdx();

		/*TIMESTAMP_TYPE*/double tsDiff ;
//        tsDiff = evntPtr->getTimeStamp() - timestamps-> operator()(evntPtr->getRowIdx(), evntPtr ->getColumnIdx() ) ;
        //tsDiff *= .000001;
	    //tsDiff = evntPtr->getTimeStamp();

		localFlw.calVelocity(tsDiff,timestamps,  worldStatus, prevWorldStatus,
		   					 evtRw, evtClm, velocity);


		//timestamps-> operator()(evntPtr->getRowIdx(), evntPtr ->getColumnIdx() ) = evntPtr->getTimeStamp();
		//timestamps ->updateSubMatrix(evntPtr->getRowIdx() - 2, evntPtr->getColumnIdx() -2, evntPtr->getTimeStamp(), 5, 5);

        if (*velocity != 0 || *(velocity +1) != 0 ) {

            vlctyBuffer.addData(evtClm - SPATIAL_MARGINE_ADDUP,
                                evtRw - SPATIAL_MARGINE_ADDUP,
                                *velocity, *(velocity + 1),
                                evntPtr -> getTimeStamp(), *(velocity + 2) );

//			if (vlctyBuffer.addDataCheckFull(evtClm - SPATIAL_MARGINE_ADDUP,
//											 evtRw - SPATIAL_MARGINE_ADDUP,
//											 *velocity, *(velocity + 1),
//											 evntPtr -> getTimeStamp(), *(velocity + 2) ) ){ //TODO
//				//Buffer is full and it should be sent to the network
//				if (outPort -> getOutputCount()){
//					VelocityBuffer & outObj = outPort->prepare();
//					outObj.setData(vlctyBuffer);
//					outPort->write();
//				}
//				vlctyBuffer.emptyBuffer();
//
//			}

		}// end if velocity != 0

	}

  	if (!vlctyBuffer.isEmpty()){
		if (outPort -> getOutputCount()){
			VelocityBuffer & outObj = outPort->prepare();
			outObj.setData(vlctyBuffer);
			outPort->write();
		}
		vlctyBuffer.emptyBuffer();
	}


}



WorldOptFlow::~WorldOptFlow(){
	cout << "world flow is closed" << endl;
}


