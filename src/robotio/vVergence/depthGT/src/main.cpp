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


#include "depthGT.h"
#include <yarp/os/all.h>


int main(int argc, char * argv[])
{

    yarp::os::Network::init();

    depthgt module;
    yarp::os::ResourceFinder rf;

    //set up the resource finder
    rf.setDefaultConfigFile("depthgt.ini"); //overridden by --from parameter
    rf.setDefaultContext("eMorph");   //overridden by --context parameter
    rf.configure(argc, argv);

    //run the module
    module.runModule(rf);

    yarp::os::Network::fini();
    return 0;
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

