// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file disparitySelectorModule.cpp
 * @brief Implementation of the disparitySelectorModule (see header file).
 */

#include "iCub/disparitySelectorModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool disparitySelectorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/disparitySelector"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    inputPortName           = rf.check("inputPortName",
			                Value("/AER:i"),
                            "Input port name (string)").asString();
    
    
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }


    /* create the thread and pass pointers to the module parameters */
    edThread = new disparitySelectorThread(robotName, configFile);
    edThread->setName(getName().c_str());
    edThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    edThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool disparitySelectorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool disparitySelectorModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    edThread->stop();
    return true;
}

bool disparitySelectorModule::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
            "size <int> <int> -to change, if allowed, size (width, height) of disparitySelector to <int> <int> \n" + 
            "place <int> -to place, if allowed, input image's center (horz, vert) in disparitySelector refernce frame \n" +
                "quit \n";
    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool disparitySelectorModule::updateModule()
{
    return true;
}

double disparitySelectorModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

