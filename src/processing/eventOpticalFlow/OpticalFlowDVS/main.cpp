/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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



#include <iostream>
#include "OpticalFlowModule.h"

using namespace std;




int main (int argc, char * argv []) {

    Network yarp;


    OpticalFlowModule optFlowModule;


    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultConfigFile("eventBasedOpticalFlow.ini"); //overriden by --from parameter
    //rf.setDefaultContext("eventOpticalFlow/conf"); //overriden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    if (!optFlowModule.configure(rf)){
        cerr << "Error in Configuring opticalFlow Module, returning" << endl;
        return -1;
    }

    optFlowModule.runModule();

    cout << "OpticalFlow Module shutting down .... " << endl;

    return 0;
}

