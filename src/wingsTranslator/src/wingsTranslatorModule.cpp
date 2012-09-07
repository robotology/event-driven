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
 * @file wingsTranslatorModule.cpp
 * @brief Implementation of the wingsTranslatorModule (see header file).
 */

#include <iCub/wingsTranslatorModule.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool wingsTranslatorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
     if(rf.check("help")) {
        printf("HELP \n");
        printf("====== \n");
        printf("--name         : changes the rootname of the module ports \n");
        printf("--config       : changes the eye config file e.g. icubEyes.ini");
        printf("--robot        : changes the name of the robot where the module interfaces to  \n"); 
        printf("--tableHeight  : changes the reference height of the plane for homography");
        printf("--kinWingsLeft : look for the kinematic constraint of the camera ");
        printf("--kinWingsRight: look for the kinematic constraint of the camera ");
        printf("press CTRL-C to continue.. \n");
        return true;
    }

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/wingsTranslator"), 
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

    /* setting the table height for homography */
    tableHeight             = rf.check("tableHeight", 
                           Value(0.12), 
                           "sets the plane z-axis height for homography (double)").asDouble();
    printf("tableHeight: %f \n", tableHeight);


    /* eyes config file for projection into the image plane */
    configName             = rf.check("config", 
                           Value("icubEyes.ini"), 
                           "Config file for intrinsic parameters (string)").asString();
    printf("configFile: %s \n", configName.c_str());
    if (strcmp(configName.c_str(),"")) {
        printf("looking for the config file \n");
        configFile=rf.findFile(configName.c_str());
        printf("config file %s \n", configFile.c_str());
        if (configFile=="") {
            printf("file not found; the program will proceed with standard values \n");
            return false;
        }
    }
    else {
        configFile.clear();
    }

    /*******************************************************************************************************/
    isOnWings = true;
    printf("trying to read the kinematic chain for the left \n");
    wingsLeftName          = rf.check("kinWingLeft", 
                                      Value("null"),   //wingsKinematic.ini"
                           "Config file for kinematics left wing (string)").asString();
    printf("wingLeftName: %s \n", wingsLeftName.c_str());
    if (strcmp(wingsLeftName.c_str(),"null")) {
        printf("looking for the wingsLeft file \n");
        wingsLeftFile=rf.findFile(wingsLeftName.c_str());
        printf("wings left file %s \n", wingsLeftFile.c_str());
        if (wingsLeftFile=="") {
            printf("ERROR: file not found; the program will proceed with standard values \n");
            isOnWings = false;
            //return false;
        }
    }
    else {
        printf("left : setting isOnWings false because not found \n");
        isOnWings = false;
        wingsLeftFile.clear();
    }

    /******************************************************************************************************/
    printf("trying to read the kinematic chain for the right \n");
    wingsRightName          = rf.check("kinWingRight", 
                                       Value("null"),   //wingsKinematic.ini"
                           "Config file for kinematics right wing (string)").asString();
    printf("wingRightName: %s \n", wingsRightName.c_str());
    if (strcmp(wingsRightName.c_str(),"null")) {
        printf("looking for the wingsRight file \n");
        wingsRightFile=rf.findFile(wingsRightName.c_str());
        printf("wings right file %s \n", wingsRightFile.c_str());
        if (wingsRightFile=="") {
            printf("ERROR: file kinematic right eye not found; the program will proceed with standard values \n");
            isOnWings = false;
            //return false;
        }
    }
    else {
        printf("left : setting isOnWings false because not found \n");
        isOnWings = false;
        wingsRightFile.clear();
    }


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

    tf = new wingsTranslatorThread();
    tf->setMapURL(mapNameComplete);
    tf->setRobotName(robotName);
    tf->setConfigFile(configFile);
    if(isOnWings) {
        printf("setting the isOnWings TRUE in the module \n");
        tf->setIsOnWings(true);
        tf->setWingsLeftFile(wingsLeftFile);
        tf->setWingsRightFile(wingsRightFile);
    }
    else {
        printf("setting the isOnWings FALSE in the module \n");
        tf->setIsOnWings(false);
    }
    tf->setTableHeight(tableHeight);
    tf->setName(getName().c_str());
    tf->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool wingsTranslatorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool wingsTranslatorModule::close() {
    handlerPort.close();
    /* stop the thread */
    tf->stop();
    return true;
}

bool wingsTranslatorModule::respond(const Bottle& command, Bottle& reply) {

    reply.clear();

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

    respondLock.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            //reply.addString("many");    // what used to work
            reply.addString("help");
            reply.addString("commands are:");
            reply.addString(" help  : to get help");
            reply.addString(" quit  : to quit the module");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" sus   ");
            reply.addString(" res   ");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString("  get3D u v left : get the 3D position using homography ");
            //reply.addString(helpMessage.c_str());
            ok = true;
        }
        break;
    case COMMAND_VOCAB_QUIT:
        rec = true;
        {
            reply.addString("quitting");
            ok = false;
        }
        break;
    
    
    case COMMAND_VOCAB_SUSPEND:
        rec = true;
        {            
            reply.addString("suspending");
            ok = true;
            //tf->suspend();
        }
        break;
    case COMMAND_VOCAB_RESUME:
        rec = true;
        {
            reply.addString("resuming");
            ok = true;
            //tf->resume();
        }
        break;
    case COMMAND_VOCAB_GET3D:
        rec = true;
        {
            //reply.addString("get3D");
            int u = command.get(1).asInt();
            int v = command.get(2).asInt();
            yarp::sig::Vector res;

            if(command.size() > 3) {
                ConstString stereoRef = command.get(3).asString();
                if(command.size() > 4) {
                    double x = command.get(4).asDouble();
                    double y = command.get(5).asDouble();
                    printf("received get3d query with u %d v %d camera %s \n", u,v, stereoRef.c_str());
                    tf->set3DTarget(x,y);
                }
                
                
                if(stereoRef == "left") {
                    res = tf->get3dWingsLeft(u,v);
                }
                else {
                    res = tf->get3dWingsRight(u,v);
                }
            }
            else {
                res = tf->get3dWingsLeft(u,v);                
            }
            
            
            reply.addDouble(res[0]);
            reply.addDouble(res[1]);
            reply.addDouble(res[2]);

            ok = true;
           
        }
        break;
    default:
        rec = false;
        ok  = false;
    }    
    
    respondLock.post();
    if (!rec){
        ok = RFModule::respond(command,reply);
    }
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
    
}

/* Called periodically every getPeriod() seconds */
bool wingsTranslatorModule::updateModule() {
    return true;
}


