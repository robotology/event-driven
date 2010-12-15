// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @brief main code to correctly start the module
 */

#include "iCub/objectInteractorModule.h" 
#include <yarp/os/all.h>
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    Network yarp;
    
    Time::turboBoost();
    oInteractorModule module; 


    ResourceFinder rf;
    rf.setVerbose(true);
    
    rf.setDefaultConfigFile("motorIF.ini"); //overridden by --from parameter
    rf.setDefaultContext("objectInteractorApp/conf");   //overridden by --context parameter

    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.setDefault("table_file","table.ini");
    rf.setDefault("homography_port","/eye2world");
    rf.setDefault("opdbServerName","/OPDB/rpc");
    rf.setDefault("name","objectInteractor");

    //rf.configure("CHRIS_ROOT",argc,argv);
    rf.configure("ICUB_ROOT", argc, argv);
 
    module.runModule(rf);
    return 0;
}


