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

#ifndef _ZYNQYARP_H
#define _ZYNQYARP_H

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include "iCub/deviceManager.h"

class device2yarp : public yarp::os::RateThread {
public:

    device2yarp();
    virtual void run();
    virtual void threadRelease();
    virtual bool threadInit(std::string moduleName = "", bool strict = false);
    bool    attachDeviceManager(deviceManager* devManager);
    bool doChannelShift;

private:

    //output port
    emorph::vBottleMimic sender;
    yarp::os::BufferedPort<emorph::vBottleMimic> portvBottle;
    //yarp::os::Port portvBottle;
    yarp::os::Stamp vStamp;

    bool strict;
    //read buffer
    //std::vector<unsigned int> deviceData;

    //device number to read from
    //int devDesc;
    aerDevManager* devManager;
    //incrementall count the number of events coming from the device
    int countAEs;
    double prevTS;
    double clockScale;

};

#endif //_ZYNQYARP_H

//----- end-of-file --- ( next line intentionally left blank ) ------------------

