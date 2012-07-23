// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file efExtractorModule.cpp
 * @brief Implementation of the efExtractorModule (see header file).
 */

#include <iCub/efExtractorModule.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool efExtractorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    if(rf.check("help")) {
        printf("Help \n");
        printf("====  \n");
        printf("--name : name of the module \n");
        printf("--mode : (intensity) mapping to be used \n");
        printf("--bottleHanlder             : the user select to send events only through bottle port esclusively  \n");
        printf("--verbose                   : saves relevant information in files \n");
        printf("press CTRL-C to continue... \n");
        return true;
    }
    

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/eventFeatureExtractor"), 
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


    /*
    * set the operating mode which correspond as well with the file map saved in conf
    */
    mapName             = rf.check("mode", 
                                   Value("intensity"), 
                                   "file map name (string)").asString();
    mapName += ".txt";
    mapNameComplete = rf.findFile(mapName.c_str());

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

    efeThread = new efExtractorThread();
    efeThread->setMapURL(mapNameComplete);
    efeThread->setName(getName().c_str());

    /* 
     *checking whether the user wants exclusively to send events as bottles
     */
    if( rf.check("bottleHandler")) {
        printf("--------------------------->set the bottleHandler flag true \n");
        efeThread->setBottleHandler(true);
    }
    else {
        printf("--------------------------->set the bottleHandler flag false \n");
        efeThread->setBottleHandler(false);
    }
    
    if(rf.check("verbose")) {
        efeThread->setVERBOSE(true);
    }
    else {
        efeThread->setVERBOSE(false);
    }

    if(!efeThread->start()) {
        return false;
    }

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool efExtractorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool efExtractorModule::close() {
    handlerPort.close();
    /* stop the thread */
    efeThread->stop();
    //delete efeThread;
    return true;
}

bool efExtractorModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
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
bool efExtractorModule::updateModule() {
    return true;
}


