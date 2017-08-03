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
    rf.setDefaultConfigFile( "vDelayControl.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}


/*////////////////////////////////////////////////////////////////////////////*/
//vParticleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool module::configure(yarp::os::ResourceFinder &rf)
{
    //administrative options
    setName((rf.check("name", yarp::os::Value("/delayControl")).asString()).c_str());
    int nthread = rf.check("threads", yarp::os::Value(2)).asInt();
    int height = rf.check("height", yarp::os::Value(240)).asInt();
    int width = rf.check("width", yarp::os::Value(304)).asInt();
    int bins = rf.check("bins", yarp::os::Value(64)).asInt();
    int maxtoproc = rf.check("maxtoproc", yarp::os::Value(500)).asInt();

    //flags
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();
    bool useroi = rf.check("useroi") &&
            rf.check("useroi", yarp::os::Value(true)).asBool();

    //filter paramters
    int rightParticles = rf.check("rParticles", yarp::os::Value(100)).asInt();
    int leftParticles = rf.check("lParticles", yarp::os::Value(0)).asInt();
    double nRandResample = rf.check("randoms", yarp::os::Value(0.0)).asDouble();

    yarp::os::Bottle * seed = rf.find("seed").asList();

    //observation parameters
    double minlikelihood = rf.check("obsthresh", yarp::os::Value(20.0)).asDouble();
    double inlierParameter = rf.check("obsinlier", yarp::os::Value(1.5)).asDouble();
    double particleVariance = rf.check("variance", yarp::os::Value(0.5)).asDouble();

    delaycontrol.initDelayControl(0.001, 30, maxtoproc);
    delaycontrol.initFilter(width, height, rightParticles,
                            bins, adaptivesampling, nthread, minlikelihood,
                            inlierParameter, nRandResample);
    if(!delaycontrol.open(getName()))
        return false;
    return delaycontrol.start();

}

/******************************************************************************/
bool module::interruptModule()
{
    delaycontrol.stop();
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


