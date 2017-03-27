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

#include "vParticleModule.h"

using ev::event;
using ev::AddressEvent;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    vParticleModule particleModule;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vParticleFilter.ini" );
    rf.configure( argc, argv );

    return particleModule.runModule(rf);
}


/*////////////////////////////////////////////////////////////////////////////*/
//vParticleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool vParticleModule::configure(yarp::os::ResourceFinder &rf)
{
    //administrative options
    setName((rf.check("name", yarp::os::Value("/vParticleFilter")).asString()).c_str());
    int nthread = rf.check("threads", yarp::os::Value(2)).asInt();

    //flags
    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    bool realtime = rf.check("realtime") &&
            rf.check("realtime", yarp::os::Value(true)).asBool();
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();
    bool useroi = rf.check("useroi") &&
            rf.check("useroi", yarp::os::Value(true)).asBool();
    int camera = 0;
    if(rf.check("camera") && rf.find("camera").asString() == "right")
        camera = 1;

    //filter paramters
    int nParticles = rf.check("particles", yarp::os::Value(50)).asInt();
    double nRandResample = rf.check("randoms", yarp::os::Value(0.02)).asDouble();
    int rate = rf.check("rate", yarp::os::Value(1000)).asDouble();

    yarp::os::Bottle * seed = rf.find("seed").asList();

    //observation parameters
    double minlikelihood = rf.check("obsthresh", yarp::os::Value(20.0)).asDouble();
    double inlierParameter = rf.check("obsinlier", yarp::os::Value(1.5)).asDouble();
    double outlierParameter = rf.check("obsoutlier", yarp::os::Value(3.0)).asDouble();
    double particleVariance = rf.check("variance", yarp::os::Value(0.5)).asDouble();


    if(!realtime) {
        particleThread = 0;
        /* USE FULL PROCESS IN CALLBACK */
        particleCallback = new vParticleReader;
        particleCallback->setObservationParameters(minlikelihood, inlierParameter,
                                                 outlierParameter);
        if(seed && seed->size() == 3) {
            std::cout << "Using initial seed location: " << seed->toString() << std::endl;
            particleCallback->setSeed(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
        }
        particleCallback->initialise(rf.check("width", yarp::os::Value(304)).asInt(),
                                     rf.check("height", yarp::os::Value(240)).asInt(),
                                     nParticles, rate, nRandResample, adaptivesampling,
                                     particleVariance, camera, useroi);

        //open the ports
        if(!particleCallback->open(getName(), strict)) {
            std::cerr << "Could not open required ports" << std::endl;
            return false;
        }
    } else {
        particleCallback = 0;
        /* USE REAL-TIME THREAD */
        particleThread = new particleProcessor(
                    rf.check("height", yarp::os::Value(240)).asInt(),
                    rf.check("width", yarp::os::Value(304)).asInt(),
                    this->getName(), strict);
        particleThread->setComputeOptions(camera, nthread, useroi);
        particleThread->setFilterParameters(nParticles, nRandResample,
                                            adaptivesampling, particleVariance);
        particleThread->setObservationParameters(minlikelihood, inlierParameter,
                                                 outlierParameter);
        if(seed && seed->size() == 3) {
            std::cout << "Using initial seed location: " << seed->toString() << std::endl;
            particleThread->setSeed(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
        }

        if(!particleThread->start())
            return false;
    }

    return true;
}

/******************************************************************************/
bool vParticleModule::interruptModule()
{
    if(particleCallback) particleCallback->interrupt();
    if(particleThread) particleThread->stop();
    std::cout << "Interrupt Successful" << std::endl;
    return true;
}

/******************************************************************************/
bool vParticleModule::close()
{
    if(particleCallback) particleCallback->close();
    std::cout << "Close Successful" << std::endl;
    return true;
}

/******************************************************************************/
bool vParticleModule::updateModule()
{
    return true;
}

/******************************************************************************/
double vParticleModule::getPeriod()
{
    return 1;

}


