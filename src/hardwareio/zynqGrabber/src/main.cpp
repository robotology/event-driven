/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
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

#include "zynqGrabberModule.h"


int main(int argc, char * argv[])
{
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("zynqGrabber.ini"); //overridden by --from parameter
    rf.setDefaultContext("eventdriven");   //overridden by --context parameter
    rf.configure(argc, argv);

	zynqGrabberModule module;
    return module.runModule(rf);

}


