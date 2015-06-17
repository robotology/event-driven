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

#include "vCircleModule.h"
#include <sstream>
//#include <opencv2/opencv.hpp>

/******************************************************************************/
bool vCircleModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vCircleFinder")
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


    //initialise the dection and tracking
    circleReader.inlierThreshold = inlierThreshold;

    circleReader.circleFinder.init(width, height, temporalWindow, obsRadius,
                                   inlierThreshold, angleThreshold,
                                   radiusThreshold);

    circleReader.circleTracker.init(procNoisePos, procNoiseRad,
                                    measNoisePos, measNoiseRad);

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
vCircleReader::vCircleReader()
{
    inlierThreshold = 5;
    periodstart = 0;
}

/******************************************************************************/
bool vCircleReader::open(const std::string &name)
{

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);

    return state1 && state2;
}

/******************************************************************************/
void vCircleReader::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/******************************************************************************/
void vCircleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();


}

/******************************************************************************/
void vCircleReader::onRead(emorph::vBottle &bot)
{

    double tstart = yarp::os::Time::now();
    
    // prepare output vBottle with address events extended with cluster ID (aec)
    // and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot;

    //create event queue
    emorph::vQueue q = bot.get<emorph::OpticalFlowEvent>();
    if(!q.size()) return;

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::OpticalFlowEvent>();
        if(!v || v->getChannel()) continue;
        //if(!v->getPolarity()) continue;


        circleFinder.addEvent(*v);
    }

    bool circlewasfound = false;
    double count = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        if(yarp::os::Time::now() - tstart > 0.0008) break;
        count++;

        emorph::AddressEvent *v = (*qi)->getAs<emorph::OpticalFlowEvent>();
        if(!v || v->getChannel()) continue;
        //if(!v->getPolarity()) continue;

        double cx, cy, cr;
        double inliers = circleFinder.flowcircle(cx, cy, cr);
        //double e1 = 12;

        if(cr < 5 || cr > 24) continue;
        if(inliers < inlierThreshold) continue;


        circlewasfound = true;
//        emorph::ClusterEventGauss circevent;
//        circevent.setStamp(v->getStamp());
//        circevent.setChannel(0);
//        circevent.setXCog(cx);
//        circevent.setYCog(cy);
//        circevent.setXSigma2(cr);
//        circevent.setYSigma2(2);
//        outBottle.addEvent(circevent);
//        continue;


//        if(false && yarp::os::Time::now() - periodstart > 3) {
//            circleFinder.flowView();
//            cv::waitKey(20);
//            periodstart = yarp::os::Time::now();
//        }

        if(!circleTracker.isActive()) {
            circleTracker.startTracking(cx, cy, cr);
            pTS = unwrap(v->getStamp());
        } else {
            double ts = unwrap(v->getStamp());
            circleTracker.predict((ts - pTS)*emorph::vtsHelper::tstosecs());
            circleTracker.correct(cx, cy, cr);
            pTS = ts;
        }

    }

    if(count != q.size()) {
        //std::cout << "Processed " << count * 100.0 / q.size() << "%" << std::endl;
    }

    double x, y, r;
    if(circlewasfound && circleTracker.getState(x, y, r)) {
        emorph::ClusterEventGauss circevent;
        circevent.setStamp(pTS);
        circevent.setChannel(0);
        circevent.setXCog(x);
        circevent.setYCog(y);
        circevent.setXSigma2(r);
        circevent.setYSigma2(2);
        outBottle.addEvent(circevent);
    }

    //send on the processed events
    outPort.write();

    double tthread = yarp::os::Time::now() - tstart;
    if(tthread > 0.001) {
        std::cout << "On Read took too long " << tthread*1000  << "ms" << std::endl;
    }

}

//empty line to make gcc happy
