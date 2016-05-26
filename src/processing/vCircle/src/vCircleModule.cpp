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
    //administrative options
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCircle")).asString();
    setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    bool parallel = rf.check("parallel");

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();


    //observation parameters
    double inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(30)).asDouble() / 100.0;

    std::string qType = rf.check("qType",
                                 yarp::os::Value("edge")).asString();

    double fifolength = rf.check("fifo", yarp::os::Value(1000.0)).asDouble();

    bool usedirected = rf.check("arc");
    int arc = rf.check("arc", yarp::os::Value(15)).asInt();
    if(!arc) usedirected = false;

    int radmin = rf.check("radmin", yarp::os::Value(10)).asInt();
    int radmax = rf.check("radmax", yarp::os::Value(35)).asInt();

    //filter parameters
//    double procNoisePos = rf.check("procNoisePos",
//                                   yarp::os::Value(5)).asDouble();

//    double procNoiseRad = rf.check("procNoiseRad",
//                                   yarp::os::Value(5)).asDouble();

//    double measNoisePos = rf.check("measNoisePos",
//                                   yarp::os::Value(5)).asDouble();

//    double measNoiseRad = rf.check("measNoiseRad",
//                                   yarp::os::Value(5)).asDouble();

    //data for experiments
    std::string datafilename = rf.check("datafile",
                                        yarp::os::Value("")).asString();

    circleReader.cObserverL =
            new vCircleMultiSize(inlierThreshold, qType, radmin, radmax,
                                 usedirected, parallel, width, height, arc, fifolength);
    circleReader.cObserverL->setChannel(0);

    circleReader.cObserverR =
            new vCircleMultiSize(inlierThreshold, qType, radmin, radmax,
                                 usedirected, parallel, width, height, arc, fifolength);
    circleReader.cObserverR->setChannel(1);

    //initialise the dection and tracking
    circleReader.inlierThreshold = inlierThreshold;

    //circleReader.circleTracker.init(procNoisePos, procNoiseRad,
    //                                measNoisePos, measNoiseRad);

    if(!circleReader.setDataWriter(datafilename)) {
        std::cerr << "Could not open datafile" << std::endl;
        return false;
    }

    //open the ports
    if(!circleReader.open(moduleName, strict)) {
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
    circleReader.close();
    yarp::os::RFModule::close();
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
    return 1;

}

/******************************************************************************/
vCircleReader::vCircleReader()
{
    inlierThreshold = 5;
    hough = false;
    timecounter = 0;
    strictness = false;
    pstampcounter = -1;
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
        this->strictness = true;
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
    std::cout << "vCircle spent " << this->timecounter
              << " seconds processing events" << std::endl;
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

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8) continue;
            int px = cx + x; int py = cy + y;
            if(py < 0 || py > 127 || px < 0 || px > 127) continue;
            image(py, 127 - px) = yarp::sig::PixelBgr(0, 0, 255);




        }
    }



}

/******************************************************************************/
void vCircleReader::onRead(emorph::vBottle &inBot)
{

    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = inBot;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);
    if(!pstamp.isValid()) pstamp = st;
    if(pstampcounter < 0) pstampcounter = st.getCount();

    //create event queue
    emorph::vQueue q = inBot.get<emorph::AddressEvent>();
    q.sort(true);

    if(!q.size()) {
        if(strictness) outPort.writeStrict();
        else outPort.write();
        return;
    }

    double t1 = yarp::os::Time::now();
    cObserverL->addQueue(q);
    cObserverR->addQueue(q);
    timecounter += yarp::os::Time::now() - t1;

    int bestx, besty, bestr;
    double bestts = unwrap(q.back()->getStamp());

    if(datawriter.is_open()) {
//        std::vector<double> percentiles  = cObserver->getPercentile(0.5, 0.1);
//        for(int i = 0; i < percentiles.size(); i += 4) {
//            datawriter << bestts << " " << (int)percentiles[i] << " "
//                       << (int)percentiles[i + 1] << " " << (int)percentiles[i + 2] << " "
//                       << percentiles[i + 3] << " " << 0 << std::endl;
//        }
        double bestscore = cObserverL->getObs(bestx, besty, bestr);
        datawriter << bestts << " " << bestx << " " << besty << " "
                   << bestr << " " << bestscore << std::endl;
        //std::cout << bestts << std::endl;
    }

    double bestScore = cObserverL->getObs(bestx, besty, bestr);

    if(bestScore > inlierThreshold) {

        //std::cout << bestx << " " << besty << " " << bestr << std::endl;
        emorph::ClusterEventGauss circevent;
        circevent.setStamp(bestts);
        circevent.setChannel(0);
        circevent.setXCog(bestx);
        circevent.setYCog(besty);
        circevent.setXSigma2(bestr);
        circevent.setYSigma2(1);
        circevent.setID(0);
        outBottle.addEvent(circevent);

    }

    bestScore = cObserverR->getObs(bestx, besty, bestr);

    if(bestScore > inlierThreshold) {

        //std::cout << bestx << " " << besty << " " << bestr << std::endl;
        emorph::ClusterEventGauss circevent;
        circevent.setStamp(bestts);
        circevent.setChannel(1);
        circevent.setXCog(bestx);
        circevent.setYCog(besty);
        circevent.setXSigma2(bestr);
        circevent.setYSigma2(1);
        circevent.setID(0);
        outBottle.addEvent(circevent);

    }

    //send on our event bottle
    if(strictness) outPort.writeStrict();
    else outPort.write();

    //send on our scope if needed
    if(scopeOut.getOutputCount()) {
        yarp::os::Bottle &scopebottle = scopeOut.prepare();
        scopebottle.clear();
        scopebottle.add(st.getCount() - pstampcounter);
        scopeOut.setEnvelope(st);
        scopeOut.write();
    }
    pstampcounter = st.getCount();

    //send on our debug image if needed
    double dstamp = st.getTime() - pstamp.getTime();
    if(houghOut.getOutputCount() && (dstamp > 0.03333 || dstamp < 0)) {
        pstamp = st;
        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = houghOut.prepare();
        image = cObserverL->makeDebugImage();
        if(bestScore > inlierThreshold) {
            drawcircle(image, bestx, besty, bestr);
        }
        houghOut.setEnvelope(st);
        houghOut.write();
        //std::cout << "Processing Time" << timecounter << ", Hough response: " << cObserver->getObs(bestx, besty, bestr) << std::endl;
    }


}
