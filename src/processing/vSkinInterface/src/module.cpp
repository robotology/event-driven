/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include "module.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    module instance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vSkinInterface.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}


/*////////////////////////////////////////////////////////////////////////////*/
//vParticleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool module::configure(yarp::os::ResourceFinder &rf)
{
    //administrative options
    setName((rf.check("name", yarp::os::Value("/skinInterface")).asString()).c_str());
    //int nthread = rf.check("threads", yarp::os::Value(1)).asInt();
    
    //flags
    //bool adaptivesampling = rf.check("adaptive") &&
    //        rf.check("adaptive", yarp::os::Value(true)).asBool();


    
    if(!skinterface.open(getName()))
        return false;
    return skinterface.start();

}

/******************************************************************************/
bool module::interruptModule()
{
    skinterface.stop();
    return true;
}

/******************************************************************************/
bool module::close()
{

    return true;
}

/******************************************************************************/
bool module::updateModule()
{
    return true;
}

/******************************************************************************/
double module::getPeriod()
{
    return 1;

}


