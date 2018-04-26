/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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

// \defgroup Modules Modules
// \defgroup vParticleFilter vParticleFilter
// \ingroup Modules
/// \brief Event-driven Detection and Tracking using a Particle Filter

#ifndef __V_PARTICLEMODULE__
#define __V_PARTICLEMODULE__

#include <yarp/os/RFModule.h>
#include "vRealTime.h"
#include "vFixedRate.h"


/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLEMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class vParticleModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vParticleReader *particleCallback;
    particleProcessor *rightThread;
    particleProcessor *leftThread;
    hSurfThread eventhandler;
    collectorPort outport;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
