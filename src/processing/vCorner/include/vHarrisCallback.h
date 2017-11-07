/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
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

/// \defgroup Modules Modules
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VHARRISCALLBACK__
#define __VHARRISCALLBACK__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <filters.h>
#include <fstream>
#include <math.h>
#include <iomanip>

class vHarrisCallback : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //data structures
    ev::temporalSurface *surfaceleft;
    ev::temporalSurface *surfaceright;

    //parameters
    int height;
    int width;
    unsigned int qlen;
    int temporalsize;
    int windowRad;
    double thresh;

    double tout;

    filters convolution;
    bool detectcorner(const ev::vQueue subsurf, int x, int y);

public:

    vHarrisCallback(int height, int width, double temporalsize, int qlen,
                    int filterSize, int windowRad, double sigma, double thresh);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);

};

#endif
//empty line to make gcc happy

