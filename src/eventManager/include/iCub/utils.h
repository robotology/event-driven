/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/PortReport.h>

#include <cv.h>

class Manager;  //forward declaration

/**********************************************************/
class PointedLocation : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CvPoint loc;
    double  rxTime;
    double  timeout;

    void onRead(yarp::os::Bottle &b);

public:
    PointedLocation();
    bool getLoc(CvPoint &loc);
};

#endif
