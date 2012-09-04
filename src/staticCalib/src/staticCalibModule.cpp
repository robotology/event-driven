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
 * @file staticCalibModule.cpp
 * @brief Implementation of the module (see header staticCalibModule.h)
 */

#include "stereoCalibModule.h"
#include <yarp/os/Stamp.h>
#include <yarp/os/Os.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

bool staticCalibModule::configure(yarp::os::ResourceFinder &rf)
{
    moduleName            = rf.check("name", 
                           Value("staticCalib"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName        = "/";
    handlerPortName       += getName(
                           rf.check("CommandPort", 
                           Value("/cmd"),
                           "Output image port (string)").asString()
                           );
    char dirName[255];
    bool proceed=true;
    string dir = rf.getContextPath().c_str();

    for (int i=1; proceed; i++)
    {
           sprintf(dirName,"%s/%s_%.5d",dir.c_str(),"calibImg",i);
           proceed=!yarp::os::stat(dirName);
           sprintf(dirName,"%s/%s_%.5d/",dir.c_str(),"calibImg",i);
     }
    
    createFullPath(dirName);

    if (!handlerPort.open(handlerPortName.c_str())) {
      cout << ": unable to open port " << handlerPortName << endl;
      return false;
    }
    attach(handlerPort);

    calibThread = new staticCalibThread(rf,&handlerPort, dirName);
    calibThread->start();

    return true;

}


bool staticCalibModule::interruptModule()
{
    calibThread->stopCalib();
    calibThread->stop();
    return true;
}


bool staticCalibModule::close()
{
    calibThread->stop();
    delete calibThread;

    return true;
}


bool staticCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    if (command.get(0).asString()=="start") {
        reply.addString("Starting Calibration...");
        calibThread->startCalib();
   }
    return true;
}

bool staticCalibModule::updateModule()
{
    return true;
}



double staticCalibModule::getPeriod()
{    
   return 0.1;
}

void staticCalibModule::createFullPath(const char* path)
{
    if (yarp::os::stat(path))
    {
        string strPath=string(path);
        size_t found=strPath.find_last_of("/");
    
        while (strPath[found]=='/')
            found--;

        createFullPath(strPath.substr(0,found+1).c_str());
        yarp::os::mkdir(strPath.c_str());
    }
}


