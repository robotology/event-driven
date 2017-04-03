/*
 * Copyright (C) 2010 eMorph Group iCub Facility
 * Authors: Arren Glover
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

#ifndef __depthgt__
#define __depthgt__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>


class depthgt : public yarp::os::RFModule {

private:

    //! the list of output ports for images
    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > depthImIn;
    yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelMono16> > depthImOut;
    yarp::os::BufferedPort< yarp::os::Bottle > depthOut;

    int roiSize;
    int roiX;
    int roiY;
    int offset;

public:

    // configure all the module parameters and return true if successful
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();         // interrupt, e.g., the ports
    virtual bool close();                   // close and shut down the modulereturn

    //when we call update module we want to send the frame on the output port
    //we use the framerate to determine how often we do this
    virtual bool updateModule();
    virtual double getPeriod();
};



#endif

