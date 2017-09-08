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
    int nthread = rf.check("threads", yarp::os::Value(1)).asInt();
    int height = rf.check("height", yarp::os::Value(240)).asInt();
    int width = rf.check("width", yarp::os::Value(304)).asInt();
    int bins = rf.check("bins", yarp::os::Value(64)).asInt();
    int maxq = rf.check("maxq", yarp::os::Value(500)).asInt();
    double gain = rf.check("gain", yarp::os::Value(0.0005)).asDouble();
    int mindelay = rf.check("mindelay", yarp::os::Value(1)).asInt();
    int qlimit = rf.check("qlimit", yarp::os::Value(0)).asInt();
    if(qlimit < 0) qlimit = 0;

    //flags
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();

    //filter paramters
    int particles = rf.check("particles", yarp::os::Value(100)).asInt();
    double nRandResample = rf.check("randoms", yarp::os::Value(0.0)).asDouble();

    yarp::os::Bottle * seed = rf.find("seed").asList();

    //observation parameters
    double minlikelihood = rf.check("obsthresh", yarp::os::Value(0.2)).asDouble();
    double inlierParameter = rf.check("obsinlier", yarp::os::Value(1.5)).asDouble();
    double particleVariance = rf.check("variance", yarp::os::Value(0.5)).asDouble();
    double trueDetectionThreshold = rf.check("truethresh", yarp::os::Value(0.35)).asDouble();

    delaycontrol.initDelayControl(gain, maxq, trueDetectionThreshold * bins, mindelay);
    delaycontrol.initFilter(width, height, particles, bins, adaptivesampling,
                            nthread, minlikelihood * bins, inlierParameter, nRandResample);
    if(seed && seed->size() == 3) {
        yInfo() << "Setting initial seed state:" << seed->toString();
        delaycontrol.setFilterInitialState(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
    }
    if(!delaycontrol.open(getName(), qlimit))
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


