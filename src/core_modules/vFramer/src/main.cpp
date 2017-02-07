/*
 * Copyright (C) 2010 iCub Facility
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


#include "vFramer.h"
#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>


int main(int argc, char * argv[])
{

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("vFramer.ini");
    rf.setDefaultContext("eventdriven");
    rf.configure(argc, argv);

    vFramerModule module;
    return module.runModule(rf);
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

