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
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCircleFinder")).asString();
    setName(moduleName.c_str());

    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();


    //observation parameters
    int obsRadius = rf.check("obsWindow", yarp::os::Value(32)).asDouble();
    int temporalWindow = rf.check("temWindow", yarp::os::Value(200000)).asDouble();
    double inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(50)).asDouble() / 100.0;
    //if(inlierThreshold > 1) inlierThreshold = 1;
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
    if(!circleReader.open(moduleName, strictness)) {
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
bool vCircleReader::open(const std::string &name, bool strictness)
{

    if(strictness) {
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);


    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);

    std::string scopePortName = "/" + name + "/scope:o";
    bool state3 = scopeOut.open(scopePortName);

    std::string houghPortName = "/" + name + "/debug:o";
    bool state4 = houghOut.open(houghPortName);

    return state1 && state2 && state3 && state4;
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
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.wrapSort();

    if(!q.size()) {
        outPort.write();
        return;
    }

    int bestx, besty, bestr; double bestinliers = 0, bestts = 0;
    double ts;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        //get the event in the correct form
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel()) continue;

        ts = unwrap(v->getStamp());
        houghFinder.addEvent(*v);

        //set inliers
        double inliers = 0;
        if(houghFinder.valid)
            inliers  = houghFinder.valc;

        if(inliers > bestinliers) {
            bestinliers = inliers;
            bestx = houghFinder.xc;
            besty = houghFinder.yc;
            bestr = houghFinder.rc;
            bestts = ts;
        }

        if(datawriter.is_open() && inliers) {
            datawriter << ts * emorph::vtsHelper::tstosecs() << " " << houghFinder.xc << " "
                       << houghFinder.yc << " " << houghFinder.rc << " " << inliers << " " << (int)v->getX()
                       << " " << (int)v->getY() << std::endl;
        }
    }

    //also add a single circle tracking position to the output
    //at the moment we are just using ClusterEventGauss
    if(bestinliers > inlierThreshold) {

        emorph::ClusterEventGauss circevent;
        circevent.setStamp(bestts);
        circevent.setChannel(0);
        circevent.setXCog(bestx);
        circevent.setYCog(besty);
        circevent.setXSigma2(bestr);
        circevent.setYSigma2(1);
        circevent.setID(0);
        outBottle.addEvent(circevent);

        if(!circleTracker.isActive()) {
            circleTracker.startTracking(bestx, besty, bestr);
        } else {
            if(pTS > bestts) pTS -= emorph::vtsHelper::maxStamp();
            circleTracker.predict((bestts - pTS)*emorph::vtsHelper::tstosecs());
            circleTracker.correct(bestx, besty, bestr);
        }
        pTS = bestts;


        double x, y, r;
        if(circleTracker.getState(x, y, r)) {
            //std::cout << x << " " << y << " " << r << std::endl;
            emorph::ClusterEventGauss circevent;
            circevent.setStamp(ts);
            circevent.setChannel(0);
            circevent.setXCog(x);
            circevent.setYCog(y);
            circevent.setXSigma2(r);
            circevent.setYSigma2(1);
            circevent.setID(1);
            outBottle.addEvent(circevent);
        }
    }

    //send on our event bottle
    outPort.write();

    if(houghOut.getOutputCount()) {
        yarp::sig::ImageOf< yarp::sig::PixelMono> &image = houghOut.prepare();
        image = houghFinder.makeDebugImage2();
        houghOut.write();
    }


}

//empty line to make gcc happy
