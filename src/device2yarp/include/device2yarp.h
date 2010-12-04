// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Charles Clerq
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
 * @file device2yarp.h
 * @brief A module that extracts independent event-driven responses coming from dvs camera Dynamic Vision Sensor, an event-based camera developed at INI)
 */

#ifndef C_DEVICE2YARP
#define C_DEVICE2YARP

/** 
 *
 * \defgroup icub_device2yarp device2yarp
 * @ingroup icub_eMorph
 *
 */

//yarp include
#include <yarp/os/all.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdlib>

#include "sending_buffer.h"

using namespace std;
using namespace yarp::os;
class C_device2yarp:public RateThread
{
public:
    C_device2yarp(bool, string);
    ~C_device2yarp();
    virtual void run();
private:
    //Declaration of the method

    //Declaration of the variables
    BufferedPort<C_sendingBuffer> port;
    FILE* raw;

    int file_desc,len,sz;
    unsigned char buffer_msg[64];
    short enabled;
	char buffer[SIZE_OF_DATA];

    int err;//détection des erreursm
    unsigned int timestamp;
    short cartX, cartY, polarity;

    unsigned int xmask;
    unsigned int ymask;
    int yshift;
    int xshift;
    int polshift;
    int polmask;
    int retinalSize;

    bool save;
};
#endif //C_DEVICE2YARP
