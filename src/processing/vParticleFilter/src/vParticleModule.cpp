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
    int height = rf.check("height", yarp::os::Value(240)).asInt();
    int width = rf.check("width", yarp::os::Value(304)).asInt();

    //flags
    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    bool realtime = rf.check("realtime") &&
            rf.check("realtime", yarp::os::Value(true)).asBool();
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();
    bool useroi = rf.check("useroi") &&
            rf.check("useroi", yarp::os::Value(true)).asBool();

    //filter paramters
    int rightParticles = rf.check("rParticles", yarp::os::Value(100)).asInt();
    int leftParticles = rf.check("lParticles", yarp::os::Value(0)).asInt();
    double nRandResample = rf.check("randoms", yarp::os::Value(0.0)).asDouble();
    int rate = rf.check("rate", yarp::os::Value(1000)).asDouble();

    yarp::os::Bottle * seed = rf.find("seed").asList();

    //observation parameters
    double minlikelihood = rf.check("obsthresh", yarp::os::Value(20.0)).asDouble();
    double inlierParameter = rf.check("obsinlier", yarp::os::Value(1.5)).asDouble();
    double outlierParameter = rf.check("obsoutlier", yarp::os::Value(3.0)).asDouble();
    double particleVariance = rf.check("variance", yarp::os::Value(0.5)).asDouble();

    particleCallback = 0;
    leftThread = 0;
    rightThread = 0;

    if(!realtime) {

        /* USE FULL PROCESS IN CALLBACK */
        particleCallback = new vParticleReader;
        particleCallback->setObservationParameters(minlikelihood, inlierParameter,
                                                 outlierParameter);
        if(seed && seed->size() == 3) {
            std::cout << "Using initial seed location: " << seed->toString() << std::endl;
            particleCallback->setSeed(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
        }
        particleCallback->initialise(width, height, rightParticles, rate,
                                     nRandResample, adaptivesampling,
                                     particleVariance, 1, useroi);

        //open the ports
        if(!particleCallback->open(getName(), strict)) {
            std::cerr << "Could not open required ports" << std::endl;
            return false;
        }
    } else {

        /* USE REAL-TIME THREAD */
        eventhandler.configure(height, width, 0.05);

        if(leftParticles) {
            leftThread = new particleProcessor(getName(), height, width, &eventhandler, &outport);
            leftThread->setComputeOptions(0, nthread, useroi);
            leftThread->setFilterParameters(leftParticles, nRandResample,
                                                adaptivesampling, particleVariance);
            leftThread->setObservationParameters(minlikelihood, inlierParameter,
                                                     outlierParameter);
            if(seed && seed->size() == 3) {
                std::cout << "Using initial seed location: " << seed->toString() << std::endl;
                leftThread->setSeed(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
            }
            if(!leftThread->start())
                return false;
        }

        if(rightParticles) {
            rightThread = new particleProcessor(getName(), height, width, &eventhandler, &outport);
            rightThread->setComputeOptions(1, nthread, useroi);
            rightThread->setFilterParameters(rightParticles, nRandResample,
                                                adaptivesampling, particleVariance);
            rightThread->setObservationParameters(minlikelihood, inlierParameter,
                                                     outlierParameter);
            if(seed && seed->size() == 3) {
                std::cout << "Using initial seed location: " << seed->toString() << std::endl;
                rightThread->setSeed(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
            }
            if(!rightThread->start())
                return false;
        }

        outport.setRate(10);
        if(!outport.open(getName() + "/vBottle:o"))
            return false;
        if(!outport.start())
            return false;
        if(!eventhandler.open(getName() + "/vBottle:i"))
            return false;
        if(!eventhandler.start())
            return false;

    }

    return true;
}

/******************************************************************************/
bool vParticleModule::interruptModule()
{
    if(!particleCallback) outport.stop();
    if(!particleCallback) eventhandler.stop();
    if(particleCallback) particleCallback->interrupt();
    if(leftThread) leftThread->stop();
    if(rightThread) rightThread->stop();

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


