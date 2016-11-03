/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Arren Glover (@itt.it)
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

#include "eventAnalysis.h"


int main(int argc, char * argv[])
{
    //i need an resource finder to configure a module
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("eMorph");           //where to look for config
    rf.setDefaultConfigFile("eventAnalysisConfig.ini");    //config file name
    rf.configure(argc,argv);                        //args overrule file

    //creating the yarp object initialised the interaction
    yarp::os::Network yarp;

    if(!yarp.checkNetwork()) {
        std::cout << "Yarp Network Failed to Initialise" << std::endl;
        return -1;
    }

    //instantiate my module
    eventStatisticsModule esm;

    return esm.runModule(rf);



    return 0;
}
