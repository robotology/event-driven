// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Arren Glover
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
 * @file main.cpp
 * @brief main code for the cartesian frame collector of asynchronous events
 */

#include "iCub/vFramer.h"
#include <yarp/os/all.h>
#include <iCub/emorph/all.h>

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    //return runImageTest(argc, argv);

    Network::init();
    
    emorph::eFramerModule module;
    //cfCollectorModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("eFramer.ini"); //overridden by --from parameter
    rf.setDefaultContext("eMorph");   //overridden by --context parameter
    rf.configure(argc, argv);
 
    module.runModule(rf);

    Network::fini();
    return 0;
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

