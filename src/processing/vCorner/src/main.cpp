/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: valentina.vasco@iit.it
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


#include "vCornerModule.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network::init();

    /* create the module */
    vCornerModule vCornerModuleInstance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    //rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "corner.ini" );
    rf.configure( argc, argv );

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    vCornerModuleInstance.runModule(rf);
    yarp::os::Network::fini();

    return 0;
}
//empty line to make gcc happy
