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
 * @file converter.h
 * @brief A class in charge of converting the input events
 */

#ifndef _CONVERTER_H
#define _CONVERTER_H

//std
#include <iostream>
#include <ctime>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>


#include <iCub/config.h>


class converter {
public:
    converter(int, int);
    ~converter();
    yarp::sig::ImageOf<yarp::sig::PixelMono16> create_frame(std::list<AER_struct>);
    void send_frame(yarp::sig::ImageOf<yarp::sig::PixelMono16>);
private:
    int sign(int);
    float mean_event(int);

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > port;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> base_img;

    int max_amount_of_event;
    int min_amount_of_event;

    int** cart_pol_acc;

    clock_t start;

    int height;
    int width;
};
#endif //_CONVERTER_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

