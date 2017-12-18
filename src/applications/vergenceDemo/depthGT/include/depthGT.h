/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: valentina.vasco@iit.it
 *           arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
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

