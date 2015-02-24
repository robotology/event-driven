/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
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


#include "vEgoMotion.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network::init();

    /* create the module */
    vEgoMotionModule vEgoMotionModuleInstance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "vEgomotion" );
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefault("name","vEgoMotion");
    rf.configure( argc, argv );

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    vEgoMotionModuleInstance.runModule(rf);
    yarp::os::Network::fini();

    return 0;
}
//empty line to make gcc happy
