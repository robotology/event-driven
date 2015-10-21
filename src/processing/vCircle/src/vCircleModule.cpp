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

/*////////////////////////////////////////////////////////////////////////////*/
//vCircleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool vCircleModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCircle")).asString();
    setName(moduleName.c_str());

    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();


    //observation parameters
    int obsRadius = rf.check("obsWindow", yarp::os::Value(32)).asDouble();
    int temporalWindow =
            rf.check("temWindow", yarp::os::Value(200000)).asDouble();
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

    std::string qType = rf.check("windowtype",
                                 yarp::os::Value("Lifetime")).asString();
    circleReader.houghFinder.qType = qType;

    bool flowhough = rf.check("flowhough",
                              yarp::os::Value(true)).asBool();

    circleReader.houghFinder.useFlow = flowhough;
    circleReader.cObserver =
            new vCircleMultiSize(qType, 10, 36, flowhough, 128, 128);

    //initialise the dection and tracking
    circleReader.inlierThreshold = inlierThreshold;

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
    scopeOut.close();
    houghOut.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/******************************************************************************/
void vCircleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    scopeOut.interrupt();
    houghOut.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();


}

/******************************************************************************/
void vCircleReader::onRead(emorph::vBottle &inBot)
{
    
    // prepare output vBottle with address events extended with cluster ID (aec)
    // and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = inBot;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);
    if(!pstamp.isValid()) pstamp = st;

    //create event queue
    emorph::vQueue q = inBot.get<emorph::AddressEvent>();
    q.wrapSort();

    if(!q.size()) {
        outPort.write();
        return;
    }

    double t1 = yarp::os::Time::now();
    cObserver->addQueue(q);
    double t2 = yarp::os::Time::now();

    int bestx, besty, bestr; double bestinliers = 0, bestts = 0;
    double ts = unwrap(q.back()->getStamp());

    houghFinder.computationtime = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

//        //get the event in the correct form
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v || v->getChannel()) continue;

//        ts = unwrap(v->getStamp());
//        double t1 = yarp::os::Time::now();

        houghFinder.addEvent(*v);
//

//        cObserver->addEvent(*v);
//

//        std::cout << t2 - t1 << " " << t3 - t2 << std::endl;

////        if(false && datawriter.is_open()) {

////            emorph::FlowEvent *fv = v->getAs<emorph::FlowEvent>();
////            if(fv) {
////                //flow event save AE plus flow velocity death
////                datawriter << 1 << " "
////                           << ts * emorph::vtsHelper::tstosecs() << " "
////                           << (int)fv->getX() << " "
////                           << (int)fv->getY() << " "
////                           << fv->getVx() << " "
////                           << fv->getVy() << " "
////                           << fv->getDeath() * emorph::vtsHelper::tstosecs()
////                           << std::endl;
////            } else {
////                //address event save AE plus 0 0 0
////                datawriter << 0 << " "
////                           << ts * emorph::vtsHelper::tstosecs() << " "
////                           << (int)v->getX() << " "
////                           << (int)v->getY() << " " << 0 << " " << 0 << " " << 0
////                           << std::endl;
////            }
////        }
    }

    double t3 = yarp::os::Time::now();

    std::cout << "Serial Hough: " << houghFinder.computationtime << std::endl;
    std::cout << "Serial: " << t3 - t2 << " & Parallel: " << t2 - t1 << std::endl;


//    bestinliers = *houghFinder.obs_max;
//    bestx = houghFinder.x_max;
//    besty = houghFinder.y_max;
//    bestr = houghFinder.r_max;
    bestts = ts;

    bestinliers = cObserver->getObs(bestx, besty, bestr);
    //std::cout << bestinliers << std::endl;

    //ts = unwrap(v->getStamp());std::cout << bestinliers << std::endl;

//    if(false && datawriter.is_open()) {
//        datawriter << ts * emorph::vtsHelper::tstosecs() << " " << bestx << " "
//                   << besty << " " << bestr << " " << bestinliers << " " << 0 << " " << 0 << std::endl;
//    }

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

//        if(false && datawriter.is_open()) {
//            datawriter << 2 << " "
//                       << bestts * emorph::vtsHelper::tstosecs() << " "
//                       << bestx << " "
//                       << besty << " "
//                       << bestr << " " << 0 << " " << 0 << std::endl;
//        }

//        if(!circleTracker.isActive()) {
//            circleTracker.startTracking(bestx, besty, bestr);
//        } else {
//            if(pTS > bestts) pTS -= emorph::vtsHelper::maxStamp();
//            circleTracker.predict((bestts - pTS)*emorph::vtsHelper::tstosecs());
//            circleTracker.correct(bestx, besty, bestr);
//        }
//        pTS = bestts;


//        double x, y, r;
//        if(circleTracker.getState(x, y, r)) {
//            //std::cout << x << " " << y << " " << r << std::endl;
//            emorph::ClusterEventGauss circevent;
//            circevent.setStamp(ts);
//            circevent.setChannel(0);
//            circevent.setXCog(x);
//            circevent.setYCog(y);
//            circevent.setXSigma2(r);
//            circevent.setYSigma2(1);
//            circevent.setID(1);
//            outBottle.addEvent(circevent);

//            if(false && datawriter.is_open()) {
//                datawriter << ts * emorph::vtsHelper::tstosecs() << " " << bestx << " "
//                           << besty << " " << bestr << " " << bestinliers << " "
//                           << x << " " << y << " " << r << std::endl;
//            }
//        }
    }

    //send on our event bottle
    outPort.writeStrict();

//    double dstamp = st.getTime() - pstamp.getTime();
//    if(houghOut.getOutputCount() && (dstamp > 0.03333 || dstamp < 0)) {
//        pstamp = st;
//        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = houghOut.prepare();
//        image = houghFinder.makeDebugImage4();
//        houghOut.setEnvelope(st);
//        houghOut.write();
//    }


}

//empty line to make gcc happy
