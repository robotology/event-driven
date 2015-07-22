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
#include <iomanip>
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
    double inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(25)).asDouble() / 100.0;
    if(inlierThreshold > 1) inlierThreshold = 1;
    double angleThreshold = rf.check("angleThreshold",
                                     yarp::os::Value(0.1)).asDouble();
    double radiusThreshold = rf.check("radiusThreshold",
                                      yarp::os::Value(1.5)).asDouble();

    //filter parameters
    double procNoisePos = rf.check("procNoisePos",
                                   yarp::os::Value(5)).asDouble();

    double procNoiseRad = rf.check("procNoiseRad",
                                   yarp::os::Value(5)).asDouble();

    double measNoisePos = rf.check("measNoisePos",
                                   yarp::os::Value(5)).asDouble();

    double measNoiseRad = rf.check("measNoiseRad",
                                   yarp::os::Value(5)).asDouble();

    //data for experiments
    std::string datafilename = rf.check("datafile",
                                        yarp::os::Value("")).asString();

    circleReader.hough = rf.check("hough",
                                  yarp::os::Value(true)).asBool();

    circleReader.houghFinder.qType = rf.check("windowtype",
                                    yarp::os::Value("Lifetime")).asString();

    circleReader.houghFinder.useFlow = rf.check("flowhough",
                                    yarp::os::Value(true)).asBool();

    //initialise the dection and tracking
    circleReader.inlierThreshold = inlierThreshold;

    circleReader.geomFinder.init(width, height, temporalWindow, obsRadius,
                                   inlierThreshold, angleThreshold,
                                   radiusThreshold);

    circleReader.circleTracker.init(procNoisePos, procNoiseRad,
                                    measNoisePos, measNoiseRad);

    if(!circleReader.setDataWriter(datafilename)) {
        std::cerr << "Could not open datafile" << std::endl;
        return false;
    }

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
    hough = false;
}

/******************************************************************************/
bool vCircleReader::setDataWriter(std::string datafilename)
{
    //only return false if we are trying to open a file and fail
    if(!datafilename.empty()) {
        datawriter.open(datafilename.c_str());
        datawriter << std::fixed << std::setprecision(6);
        return datawriter.is_open();
    }

    return true;
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
    
    // prepare output vBottle with address events extended with cluster ID (aec)
    // and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot;

    //create event queue
    emorph::vQueue q = bot.getSorted<emorph::AddressEvent>();

    bool circlewasfound = false;
    double count = 0, potential = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        //get the event in the correct form
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel()) continue;
        if(!hough) geomFinder.addEvent(*v);

        potential++; //increment our records of possible v's to process

        //if we already have new data to read we need to finish ASAP
        if(getPendingReads()) continue;
        count++; //increment our records of v's processed

        //process the observation
        double cx, cy, cr;

        double inliers = 0;
        if(hough) {
            houghFinder.addEvent(*v);
            if(!houghFinder.found) continue;
            //if(houghFinder.valc < inlierThreshold) continue;
            if(houghFinder.valc < 6.28 * houghFinder.rc * inlierThreshold) continue;
            cx = houghFinder.xc;
            cy = houghFinder.yc;
            cr = houghFinder.rc;


        } else {
            inliers = geomFinder.flowcircle(cx, cy, cr);
            if(inliers < 6.28 * cr * inlierThreshold) continue;
            //if(cr < 5 || cr > 24) continue;
            //if(inliers < inlierThreshold) continue;
        }

        circlewasfound = true;

        //update the filter given the observation
        double ts = unwrap(v->getStamp());
        if(!circleTracker.isActive()) {
            circleTracker.startTracking(cx, cy, cr);
        } else {
            circleTracker.predict((ts - pTS)*emorph::vtsHelper::tstosecs());
            circleTracker.correct(cx, cy, cr);
        }
        pTS = ts;

        //write analysis data if needed
        if(datawriter.is_open()) {
            datawriter << ts * emorph::vtsHelper::tstosecs() << " " << cx << " "
                       << cy << " " << cr << " ";
            double kx, ky, kr;
            circleTracker.getState(kx, ky, kr);
            datawriter << kx << " " << ky << " " << kr << std::endl;
        }

    }

    //at the end of the bottle say how many events were processed
//    if(potential != 0 && count != potential) {
//        std::cout << "Processed " << count * 100.0 / potential << "%" << std::endl;
//    }

    //also add a single circle tracking position to the output
    //at the moment we are just using ClusterEventGauss
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

    //send on our event bottle
    outPort.write();

}

//empty line to make gcc happy
