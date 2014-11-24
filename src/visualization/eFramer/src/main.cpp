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
 * @brief main code for the cartesian frame collector of asynchronous events
 */

#include "iCub/eFramer.h"
//#include "iCub/cfCollectorModule.h"
#include <yarp/os/all.h>
#include <iCub/emorph/all.h>

int runImageTest(int argc, char * argv[]) {

    std::cout << "Starting Frame Testing" << std::endl;

    emorph::eAddressFrame eaf(128, 128);
    eaf.setPublishSize(512, 512);
    emorph::AddressEvent ae;



    std::cout << "Adding Events:" << std::endl;

    ae.setChannel(0);
    ae.setPolarity(0);
    ae.setX(20);
    ae.setY(30);
    ae.setStamp(1);
    eaf.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;


    ae.setChannel(0);
    ae.setPolarity(0);
    ae.setX(50);
    ae.setY(50);
    ae.setStamp(2);
    eaf.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;


    ae.setChannel(0);
    ae.setPolarity(0);
    ae.setX(61);
    ae.setY(69);
    ae.setStamp(3);
    eaf.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;

    ae.setChannel(0);
    ae.setPolarity(0);
    ae.setX(120);
    ae.setY(109);
    ae.setStamp(4);
    eaf.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;

    eaf.publish();

    return 0;
}

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    return runImageTest(argc, argv);

    Network yarp;
    
    Time::turboBoost();
    emorph::eFramerModule module;
    //cfCollectorModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("cartesianFrameCollector.ini"); //overridden by --from parameter
    rf.setDefaultContext("dvsGrabber/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    module.runModule(rf);
    return 0;
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

