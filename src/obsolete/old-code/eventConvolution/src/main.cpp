// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email: shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file main.cpp
 * @brief main code for the event convolution module; this is part of the eMorph.
 */



#include "iCub/eventConvolutionModule.h" 
//#include <ippi.h>
//#include <ippcore.h>

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    /* important, this effectively disables the OMP library parallelization in the IPP */
    //ippSetNumThreads(1);

    Network yarp;
    eventConvolutionModule module; 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("eventConvolutionModule.ini"); //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    module.runModule(rf);
    return 0;
}


