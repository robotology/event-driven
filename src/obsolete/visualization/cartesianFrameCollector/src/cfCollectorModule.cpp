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
 * @file cfCollectorModule.cpp
 * @brief Implementation of the cfCollectorModule (see header file).
 */

#include <iCub/cfCollectorModule.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool cfCollectorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    if(rf.check("help")) {
        printf("HELP \n");
        printf("=======");
        printf("--name             (string) : specifies the rootname\n");
        printf("--robot            (string) : indicates the name of the robot to connect to\n");
        printf("--retinalSize      (int)    : defines the dimension of the retina (input)\n");
        printf("--responseGradient (int)    : the increment for any single event in the register\n");
        printf("--sychPeriod       (int)    : period for synchronization of the variable lastTimestamp\n");
        printf("--windowSize       (int)    : size of the window where events are collected \n ");
        printf("--stereo                    : if present both left and right events are represented \n ");
        printf("--evType           (string) : specifies the type of events to be added to the image (cle, cleg, ofe, etc.)\n");   
        printf("--cleMax           (int)    : maximum number of Cluster Events to be added \n");   
        printf("--bottleHanlder             : the user select to send events only through bottle port esclusively  \n");
        printf("--verbose         : enable debug savings of events in files");
        printf("\n press CTRL-C to continue \n");
        return true;
    }



    printf("initialization of the main thread \n");
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/cartesianFrameCollector"), 
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
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }
    printf("attaching handler port2 \n");
    attach(handlerPort);                  // attach to port
    
    // --------------------------------------------
    printf("starting cfCollector Thread \n");
    cfThread=new cfCollectorThread();
    cfThread->setName(getName().c_str());
    
    printf("name of the cfThread correctly set \n");

    /*
    * set the period between two successive synchronisations between viewer and events
    */
    synchPeriod            = rf.check("synchPeriod", 
                           Value(10000), 
                           "synchronisation period (int)").asInt();
    printf("set synchPeriod; found value %d \n", synchPeriod);
    cfThread->setSynchPeriod(synchPeriod);

    /*
    * set the windowSize (dimension of temporal window collected events)
    */
    windowSize            = rf.check("windowSize", 
                           Value(10), 
                           "windowSize (int)").asInt();
    printf("looking for the windowSize; found value %d \n", windowSize);
    cfThread->setWindowSize(windowSize);
    
    /*
    * set the retinaSize (considering squared retina)
    */
    printf("looking for the retinalSize \n");
    retinalSize            = rf.check("retinalSize", 
                           Value(128), 
                           "retinalSize (int)").asInt();
    cfThread->setRetinalSize(retinalSize);
    
    /*
    * set the retinaSize (considering squared retina)
    */
    printf("looking for the responseGradient \n");
    responseGradient       = rf.check("responseGradient", 
                           Value(127), 
                           "responseGradient (int)").asInt();
    cfThread->setResponseGradient(responseGradient);

    
    /* 
     *checking whether the module synchronizes with single camera or stereo camera
     */
    if( rf.check("stereo")) {
        cfThread->setStereo(true);
    }
    else {
        cfThread->setStereo(false);
    }

    /* set the type of event to be plotted */
    evType            = rf.check("evType", 
                           Value("ae"), 
                           "evType (string)").asString();
    fprintf(stdout,"add %s event for plotting \n", evType.c_str());
    cfThread->setType(evType);

    /*
    * set the maximum number of cluster events to be added to the image
    */
    cleMax            = rf.check("cleMax", 
                           Value(10), 
                           "cleMax (int)").asInt();
    cfThread->setCleMax(cleMax);


     /* 
     *checking whether the user wants exclusively to send events as bottles
     */
    if( rf.check("bottleHandler")) {
        printf("set the bottleHandler flag true \n");
        cfThread->setBottleHandler(true);
    }
    else {
        printf("set the bottleHandler flag false \n");
        cfThread->setBottleHandler(false);
    }

    /* 
     *checking whether the user wants exclusively to send events as bottles
     */
    if( rf.check("verbose")) {
        printf("set the verbose mode for all the components \n");
        cfThread->setVerbose(true);
    }
    else {
        printf("verbose mode deactivated \n");
        cfThread->setVerbose(false);
    }

    /* 
     *set option for mapping three states into 3baseline graylevels
     */
    if( rf.check("tristate")) {
        cfThread->setTristate(true);
    }
    else {
        cfThread->setTristate(false);
    }


    /* 
     *checking whether the module synchronizes with single camera or stereo camera
     */
    printf("Looking for asvMode ... \n");
    if( rf.check("asvMode")) {
        printf("setting asvMode = true, dvsMode = false \n");
        cfThread->setASVMode(true);
        cfThread->setDVSMode(false);
    }
    else {
        printf("setting asvMode = false, dvsMode = false \n");
        cfThread->setASVMode(false);
    }

    
    /* checking whether the module synchronizes with single camera or stereo camera
     */
    if( rf.check("dvsMode")) {
        cfThread->setDVSMode(true);
        cfThread->setASVMode(false);
    }
    else {
        cfThread->setDVSMode(false);
    }

    /**
     * checking whether the viewer represent log-polar information
     */
    if( rf.check("logpolar")) {
        cfThread->setLogPolar(true);
    }
    else {
        cfThread->setLogPolar(false);
    }
    cfThread->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool cfCollectorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool cfCollectorModule::close() {
    handlerPort.close();
    /* stop the thread */
    cfThread->stop();
    printf("stopped the collector thread \n");
    //delete cfThread;
    return true;
}

bool cfCollectorModule::respond(const Bottle& command, Bottle& reply) {
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
bool cfCollectorModule::updateModule() {
    return true;
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

