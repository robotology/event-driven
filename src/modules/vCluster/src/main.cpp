/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi and Arren Glover
 * email:  chiara.bartolozzi@iit.it, arren.glover@iit.it
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
