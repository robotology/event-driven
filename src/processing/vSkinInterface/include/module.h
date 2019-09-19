/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

// \defgroup Modules Modules
// \defgroup vParticleFilter vParticleFilter
// \ingroup Modules
// \brief tracks targets using a particle filter

#ifndef __V_SKINMODULE__
#define __V_SKINMODULE__

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <vector>
//#include "vSkinInterface.h"


/*////////////////////////////////////////////////////////////////////////////*/
//VSKINMODULE
/*////////////////////////////////////////////////////////////////////////////*/
class module : public yarp::os::RFModule
{

    //skinInterface skinterface;
    ev::vReadPort< std::vector<ev::SkinEvent> > skinevents_in;
    ev::vReadPort< std::vector<ev::SkinSample> > skinsamples_in;
    yarp::os::BufferedPort< yarp::os::Bottle > scope_out;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
