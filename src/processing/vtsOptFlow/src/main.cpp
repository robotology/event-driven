/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author: Charles Clercq, edited by Valentina Vasco (01/15)
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

/*
 * @file main.cpp
 * @brief main code for the computation of the optical flow
 */

#include "vtsOptFlow.hpp"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network::init();

    /* instantiate the module */
    vtsOptFlow module;

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setDefaultConfigFile("opticalflow.ini");    //overridden by --from parameter
    rf.setDefaultContext("eMorph");    //overridden by --context parameter
    rf.configure(argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    /* deinitilize yarp network */
    Network::fini();

    return 0;
}

//----- end-of-file --- ( next line intentionally left blank ) ------------------
