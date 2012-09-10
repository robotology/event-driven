// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file velocityExtractorModule.cpp
 * @brief Implementation of the velocityExtractorModule (see header file).
 */

#include <iCub/velocityExtractorModule.h>

// general command vocab's
#define COMMAND_VOCAB_IS                 VOCAB2('i','s')
#define COMMAND_VOCAB_OK                 VOCAB2('o','k')

#define COMMAND_VOCAB_SET                VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET                VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN                VOCAB3('r','u','n')
#define COMMAND_VOCAB_SUSPEND            VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME             VOCAB3('r','e','s')
#define COMMAND_VOCAB_FIX                VOCAB3('f','i','x')
#define COMMAND_VOCAB_EGO                VOCAB3('E','G','O')

#define COMMAND_VOCAB_HELP               VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_SEEK               VOCAB4('s','e','e','k')
#define COMMAND_VOCAB_CENT               VOCAB4('c','e','n','t')
#define COMMAND_VOCAB_STOP               VOCAB4('s','t','o','p')


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool velocityExtractorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    printf("initialization of the main thread \n");
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/velocityExtractor"), 
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
    robotPortName         = "/" + robotName + "/head";\
    
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
    esThread=new velocityExtractorThread();
    esThread->setName(getName().c_str());
    
    printf("name of the cfThread correctly set \n");

    /*
    * set the period between two successive synchronisations between viewer and events
    */
    synchPeriod            = rf.check("synchPeriod", 
                           Value(10000), 
                           "synchronisation period (int)").asInt();
    //cfThread->setSynchPeriod(synchPeriod);

    
    /*
    * set the retinaSize (considering squared retina)
    */
    printf("looking for the retinalSize \n");
    retinalSize            = rf.check("retinalSize", 
                           Value(128), 
                           "retinalSize (int)").asInt();
    //cfThread->setRetinalSize(retinalSize);

    
    /*
    * set the retinaSize (considering squared retina)
    */
    printf("looking for the responseGradient \n");
    responseGradient       = rf.check("responseGradient", 
                           Value(127), 
                           "responseGradient (int)").asInt();
    //cfThread->setResponseGradient(responseGradient);

    esThread->start();
    esThread->suspend();


    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool velocityExtractorModule::interruptModule() {
    handlerPort.interrupt();
    esThread->interrupt();
    return true;
}

bool velocityExtractorModule::close() {
    handlerPort.close();
    printf("General handler port closed \n");
    
    
    /* stop the thread */
    esThread->stop();
    printf("stopped the collector thread \n");
    
    //delete cfThread;
    return true;
}

bool velocityExtractorModule::respond(const Bottle& command, Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?
    
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
    

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            //reply.addString();
            reply.addString("set fn \t: general set command ");
            reply.addString("get fn \t: general get command ");
            //reply.addString();

            

            reply.addString("sus  \t : suspending");
            reply.addString("res  \t : resuming");
            //reply.addString();


            ok = true;
        }
        break;
    case COMMAND_VOCAB_SUSPEND:
        rec = true;
        {
            esThread->suspend();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_RESUME:
        rec = true;
        {
            esThread->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_EGO:
        rec = true;
        {
            printf("case position %d \n",COMMAND_VOCAB_EGO );
            double egoMotionU = command.get(1).asDouble();
            double egoMotionV = command.get(2).asDouble();
            if((egoMotionU == 0) && (egoMotionV == 0)) {
                Time::delay(2.0);
            }
            esThread->setEgoMotion(egoMotionU, egoMotionV);
            ok = true;
        }
        break;
        
    default: {
        
    }
        break;    
    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool velocityExtractorModule::updateModule() {
    return true;
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

