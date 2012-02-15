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

#ifndef WORLDFLOW_H_
#define WORLDFLOW_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>

#include "MyMatrix.h"
#include "param.h"
#include "LKLocalFlow.h"
#include "AERGrabber.h"
#include "VelocityBuffer.h"

using namespace yarp::os;

class WorldOptFlow {
    LKLocalFlow   localFlw;

    MyMatrix<POLARITY_TYPE> * worldStatus;          /* keep track of the scene status,
                                                       by accumulating the event polarities */
    MyMatrix<POLARITY_TYPE> * prevWorldStatus;      /* previous status of the scene,
                                                       is used to calculate temporal derivative */

    MyMatrix<TIMESTAMP_TYPE> * timestamps;          /* stores the the last event's timestamp
                                                        at each pixel*/

    AERGrabber * inPort;                            /* is inherited from <BufferedPort> and
                                                       received input(events) from Camera */

    BufferedPort<VelocityBuffer> * outPort;
    BufferedPort<Bottle> *outBPort;

    CameraEvent *** eventBuffers;                   /*2-Dimensional Array of  "pointers to CameraEvent"-
                                                     Each row represent a bag of events which arrived in a closed interval */
    int * eventNosBuffer;                           /*keep number of events in each row of eventBuffer*/

    int   eventBuffersSize;                         /*size of 'eventBuffers' & 'eventNosBuffer'*/

    void updtWrldStus();  //void updtWrldStus(CameraEvent **, int);
    void calVelocities(CameraEvent **, int);
    void directWrldStus(CameraEvent ** , int );

public:

    WorldOptFlow( AERGrabber * inPortPtr,
               BufferedPort<VelocityBuffer> * outFlowPort,  BufferedPort<Bottle> * outBttlwPort,
               MyMatrix<POLARITY_TYPE> * wStatus, MyMatrix<POLARITY_TYPE> * pWStatus,
               MyMatrix<TIMESTAMP_TYPE> * ts);

    void initialize(int step);

    void run ();

    ~WorldOptFlow();
};


#endif /* WORLDFLOW_H_ */
