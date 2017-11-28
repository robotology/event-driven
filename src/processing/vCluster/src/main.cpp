/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
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


#include "eventClustering.h"

using namespace yarp::os;

int main(int argc, char * argv[])
{
    yarp::os::Network yarp;


    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose( false );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "cluster.ini" );
    rf.configure( argc, argv );


    /* create the module */
    EventClustering eventClustering;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return eventClustering.runModule(rf);
}
//empty line to make gcc happy
