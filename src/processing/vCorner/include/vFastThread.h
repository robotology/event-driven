/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco based on Elias Mueggler's implementation
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

#ifndef __VFASTTHREAD__
#define __VFASTTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <fstream>
#include <math.h>

class vFastThread : public yarp::os::Thread
{
private:

    //thread for queues of events
    ev::queueAllocator inputPort;

    //ports
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //data structures
    yarp::sig::ImageOf< yarp::sig::PixelInt > surfaceOnL;
    yarp::sig::ImageOf< yarp::sig::PixelInt > surfaceOfL;
    yarp::sig::ImageOf< yarp::sig::PixelInt > surfaceOnR;
    yarp::sig::ImageOf< yarp::sig::PixelInt > surfaceOfR;

    //thread for the output
    ev::collectorPort outthread;

    //synchronising value
    yarp::os::Stamp yarpstamp;

    //to unwrap the timestamp
    ev::vtsHelper unwrapper;

    //pixels on circle
    int circle3[16][2] =
    {
        {0, 3},
        {1, 3},
        {2, 2},
        {3, 1},
        {3, 0},
        {3, -1},
        {2, -2},
        {1, -3},
        {0, -3},
        {-1, -3},
        {-2, -2},
        {-3, -1},
        {-3, 0},
        {-3, 1},
        {-2, 2},
        {-1, 3}
    };

    int circle4[20][2] =
    {
        {0, 4},
        {1, 4},
        {2, 3},
        {3, 2},
        {4, 1},
        {4, 0},
        {4, -1},
        {3, -2},
        {2, -3},
        {1, -4},
        {0, -4},
        {-1, -4},
        {-2, -3},
        {-3, -2},
        {-4, -1},
        {-4, 0},
        {-4, 1},
        {-3, 2},
        {-2, 3},
        {-1, 4}
    };

    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    double gain;

    void getCircle3(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p3)[16], int (&circle3)[16][2]);
    void getCircle4(yarp::sig::ImageOf<yarp::sig::PixelInt> *cSurf, int x, int y, unsigned int (&p4)[20], int (&circle4)[20][2]);
    bool detectcornerfast(unsigned int patch3[16], unsigned int patch4[20]);

public:

    vFastThread(unsigned int height, unsigned int width, std::string name, bool strict, double gain);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();

};


#endif
//empty line to make gcc happy
