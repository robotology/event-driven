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
 * @file device2yarp.h
 * @brief Definition of the ratethread that receives events from DVS camera
 */

#ifndef _DEVICE2YARP_H
#define _DEVICE2YARP_H

#define __STDC_LIMIT_MACROS

//yarp include
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <fstream>

//within the emorph project includes
#include <iCub/emorph/eventBuffer.h>
#include <iCub/emorph/eventBottle.h>
#include <iCub/emorph/all.h>

class device2yarp : public yarp::os::RateThread {
public:
    device2yarp(int file_desc);
    ~device2yarp();
    virtual void run();
    virtual void threadRelease();
    virtual bool threadInit(std::string moduleName = "");
  
private:
    yarp::os::BufferedPort<emorph::vBottle> portvBottle;
    yarp::os::Stamp vStamp;
    
    std::vector<unsigned int> deviceData;
    
    double startInt;                                // time variable for the start of acquisition
    double stopInt;                                 // time variable for the end of acquisition
//    double maxCountInWraps;
//    double minCountInWraps;
//    double maxRate;
//    double minRate;
    
    int countAEs,countErrors;
    
    int file_desc,len,sz;
    
    bool wrapOccured;                       // flag that indicates if in the block of events a TS_WA is present

    std::stringstream str_buf;

};

#endif //_DEVICE2YARP_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

