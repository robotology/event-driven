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

#include "vHoughModule.h"
#include <sstream>
//#include <opencv2/opencv.hpp>

/******************************************************************************/
bool vCircleModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vHough")
                                      ).asString();
    setName(moduleName.c_str());

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();


    //observation parameters
    int obsRadius = rf.check("obsWindow", yarp::os::Value(32)).asDouble();
    int temporalWindow = rf.check("temWindow", yarp::os::Value(200000)).asDouble();
    int inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(5)).asInt();
    double angleThreshold = rf.check("angleThreshold",
                                     yarp::os::Value(0.1)).asDouble();
    double radiusThreshold = rf.check("radiusThreshold",
                                      yarp::os::Value(1.5)).asDouble();

    //filter parameters
    double procNoisePos = rf.check("procNoisePos",
                                   yarp::os::Value(5)).asDouble();

    double procNoiseRad = rf.check("procNoiseRad",
                                   yarp::os::Value(2)).asDouble();

    double measNoisePos = rf.check("measNoisePos",
                                   yarp::os::Value(5)).asDouble();

    double measNoiseRad = rf.check("measNoiseRad",
                                   yarp::os::Value(5)).asDouble();


    //open the ports
    if(!circleReader.open(moduleName)) {
        std::cerr << "Could not open required ports" << std::endl;
        return false;
    }

    return true ;
}

/******************************************************************************/
bool vCircleModule::interruptModule()
{
    circleReader.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vCircleModule::close()
{
    std::cout << "Closing vCircleModule" << std::endl;
    circleReader.close();
    yarp::os::RFModule::close();
    std::cout << "Closed vCircleModule" << std::endl;
    return true;
}

/******************************************************************************/
bool vCircleModule::updateModule()
{

    return true;
}

/******************************************************************************/
double vCircleModule::getPeriod()
{
    return 0.3;

}

/******************************************************************************/
// vHoughReader
/******************************************************************************/
vHoughReader::vHoughReader()
{
    pTS = 0;
    tracker.init(5, 5, 5, 5);
}

bool vHoughReader::open(const std::string &name)
{

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);

    std::string debugPortName = "/" + name + "/debugImage";
    bool state3 = debugPort.open(debugPortName);

    return state1 && state2 && state3;
}

/******************************************************************************/
void vHoughReader::close()
{
    //close ports
    outPort.close();
    debugPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/******************************************************************************/
void vHoughReader::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    debugPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();


}

/******************************************************************************/
void vHoughReader::onRead(emorph::vBottle &bot)
{
    
    bool circleFoundThisBottle = false;
    // prepare output vBottle with address events extended with cluster ID (aec)
    // and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot;

    //create event queue
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        if(getPendingReads()) break;
        if((*qi)->getChannel() != 0) continue;
        circleFinder.addEventLife(**qi);

        if(!circleFinder.found || circleFinder.valc < 30) continue;
        circleFoundThisBottle = true;
        if(!tracker.isActive()) {
            tracker.startTracking(circleFinder.xc, circleFinder.yc, circleFinder.rc);
        } else {
            tracker.predict(((*qi)->getStamp() - pTS) * emorph::vtsHelper::tstosecs());
            tracker.correct(circleFinder.xc, circleFinder.yc, circleFinder.rc);
        }

        pTS = (*qi)->getStamp();

        std::cout << circleFinder.valc << std::endl;


    }

    if(circleFoundThisBottle) {
        emorph::ClusterEventGauss detection;
        detection.setChannel(0);
        detection.setPolarity(1);
        detection.setStamp(pTS);

        double x, y, r;
        tracker.getState(x, y, r);
        detection.setXCog(x);
        detection.setYCog(y);
        detection.setXSigma2(r);
        detection.setYSigma2(2);
        outBottle.addEvent(detection);
    }
    outPort.write();
    yarp::sig::ImageOf<yarp::sig::PixelMono> &image = debugPort.prepare();
    image = circleFinder.makeDebugImage(10);
    debugPort.write();


}

//empty line to make gcc happy
