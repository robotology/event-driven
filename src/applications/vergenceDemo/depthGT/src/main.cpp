/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: valentina.vasco@iit.it
 *           arren.glover@iit.it
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

