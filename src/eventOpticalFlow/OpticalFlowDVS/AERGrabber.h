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

#ifndef AERGRABBER_H_
#define AERGRABBER_H_

#include <math.h>

#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventConversion.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <iostream>

#include "CameraEvent.h"
#include "MyMatrix.h"
#include "param.h"

#define BUFFER_SIZE 60000

class AERGrabber : public yarp::os::BufferedPort<eventBuffer>{


    CameraEvent * evntBuffer [BUFFER_SIZE]; /*Buffer of events received from one of the eyes*/
    int firstIdx;                         /*index of the earliest event in the evntBuffer*/
    int lastIdx;                          /* index of the first free space in the evntBuffer*/
    yarp::os::Semaphore eBufferMutex;     /* a semaphore to access evntBuffer and firstIdx and lastIdx
                                              -- is used in onRead and getEvents function*/

    yarp::os::Semaphore * evntsMutex;


    MyMatrix<TIMESTAMP_TYPE> offTimestamps;          /* stores the the last event's timestamp
                                                        at each pixel*/

    MyMatrix<TIMESTAMP_TYPE> onTimestamps;          /* stores the the last event's timestamp
                                                            at each pixel*/


    unsigned long wrapAddup;
    unsigned long lastFrameTS;
    unsigned long frameInv;

public:

    AERGrabber(yarp::os::Semaphore * sigEvents, unsigned long frameInv);

    ~AERGrabber();

    virtual void onRead(eventBuffer & eBuffer);

    CameraEvent ** getEvents(int & evenNo);

    bool isReliableEvent (short row, short clmn, short polarity, unsigned long timestamp);

};


#endif /* AERGRABBER_H_ */
