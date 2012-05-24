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



#ifndef OPTICALFLOWMODULE_H_
#define OPTICALFLOWMODULE_H_

#include <string>
#include <iostream>
#include <math.h>


#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <iCub/emorph/eventBuffer.h>
#include <yarp/sig/Matrix.h>

#include "param.h"
#include "WorldOptFlow.h"
#include "AERGrabber.h"
#include "MyMatrix.h"
#include "VelocityBuffer.h"


using namespace yarp::os;
using namespace emorph::ebuffer;

class OpticalFlowModule : public RFModule{
    short retinaSizeR;
    short retinaSizeC;

    //short nghbrRadius;
    //short nghbrWindow;

    unsigned long frameInv;

    MyMatrix<POLARITY_TYPE> worldStatus;          /* keep track of the scene status,
                                                     by accumulating the event polarities */
    MyMatrix<POLARITY_TYPE> prevWorldStatus;      /* previous status of the scene,
                                                     is used to calculate temporal derivative */


    MyMatrix<TIMESTAMP_TYPE> timestamps;          /* stores the the last event's timestamp
                                                     at each pixel*/

    AERGrabber * inputPort;                        /*is inherited from <BufferedPort> and
                                                   received input(events) from Camera */



    yarp::os::Semaphore * evntsMutex;


    BufferedPort<VelocityBuffer> outFlowPort;
    BufferedPort<Bottle> bottleFlowPort;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > outWrldMdlPort;


    WorldOptFlow * wrldFlw;

    void worldStatusRenderer();

    //void eventsHandler();     //TODO : input & comment

public:
    OpticalFlowModule();

    bool configure(ResourceFinder & rf);

    bool interruptModule();

    bool close();

    bool updateModule();

    double getPeriod();

    virtual ~OpticalFlowModule();
};

#endif /* OPTICALFLOWMODULE_H_ */
