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

using ev::event;

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
    bool singleq = rf.check("everyevent") &&
            rf.check("everyevent", yarp::os::Value(true)).asBool();
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
    int arc = rf.check("arc", yarp::os::Value(1)).asInt();
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
    circleReader.setSingleQ(singleq);

    //circleReader.circleTracker.init(procNoisePos, procNoiseRad,
    //                                measNoisePos, measNoiseRad);

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
    singleq = false;
    tsoffset = 0;
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
    bool state1 = yarp::os::BufferedPort<ev::vBottle>::open(inPortName);


    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);

    std::string scopePortName = "/" + name + "/scope:o";
    bool state3 = scopeOut.open(scopePortName);

    std::string houghPortName = "/" + name + "/debug:o";
    bool state4 = houghOut.open(houghPortName);

    if(!dumpOut.open("/" + name + "/dump:o")) return false;

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
    dumpOut.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

}

/******************************************************************************/
void vCircleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    scopeOut.interrupt();
    houghOut.interrupt();
    dumpOut.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();


}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8) continue;
            int px = cx + x; int py = cy + y;
            if(py < 0 || py > image.height()-1 || px < 0 || px > image.width()-1) continue;
            image(py, image.width() - px) = yarp::sig::PixelBgr(0, 0, 255);

        }
    }



}

/******************************************************************************/
void vCircleReader::onRead(ev::vBottle &inBot)
{
    // ///////////////////
    // get the data & set-up
    // ///////////////////

    ev::vBottle &outBottle = outPort.prepare();
    outBottle = inBot;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);
    if(!pstamp.isValid()) pstamp = st;
    if(pstampcounter < 0) pstampcounter = st.getCount();

    //create event queue
    ev::vQueue q = inBot.get<ev::AddressEvent>();
    q.sort(true);

    if(!q.size()) {
        if(strictness) outPort.writeStrict();
        else outPort.write();
        return;
    }

    if(tsoffset == 0)
        tsoffset = yarp::os::Time::now() - st.getTime();

    // ///////////////////
    // processing & data dumping if required
    // ///////////////////

    if(singleq) {
        double bestScore;
        int bestx, besty, bestr;
        ev::vQueue singleQ; singleQ.push_back(q.front());
        for(unsigned int i = 0; i < q.size(); i++) {
            //singleQ.push_back(q[i]);
            singleQ[0] = q[i];
            if(q[i]->getChannel() == 0) {
                cObserverL->addQueue(singleQ);
                bestScore = cObserverL->getObs(bestx, besty, bestr);
            } else {
                cObserverR->addQueue(singleQ);
                bestScore = cObserverR->getObs(bestx, besty, bestr);
            }
            //save the results
            if(dumpOut.getOutputCount()) {
                yarp::os::Bottle &dumper = dumpOut.prepare();
                dumper.clear();
                dumper.addDouble(yarp::os::Time::now() - tsoffset);
                dumper.addInt(singleQ[0]->getStamp());
                dumper.addInt(q[i]->getChannel());
                dumper.addInt(bestx);
                dumper.addInt(besty);
                dumper.addInt(bestr);
                dumper.addDouble(bestScore);
                dumpOut.setEnvelope(st);
                dumpOut.writeStrict();
            }
            singleQ[0] = q.front();
            //singleQ.clear();
        }
    } else {
        cObserverL->addQueue(q);
        cObserverR->addQueue(q);
    }

    // ///////////////////
    // send the results through in vBottle
    // ///////////////////
    int bestxL, bestyL, bestrL;
    double bestScoreL = cObserverL->getObs(bestxL, bestyL, bestrL);

    if(bestScoreL > inlierThreshold) {

        //std::cout << bestx << " " << besty << " " << bestr << std::endl;
        event<ev::ClusterEventGauss> circevent = event<ev::ClusterEventGauss>(new ev::ClusterEventGauss());

        circevent->setStamp(q.back()->getStamp());
        circevent->setChannel(0);
        circevent->setXCog(bestxL);
        circevent->setYCog(bestyL);
        circevent->setXSigma2(bestrL);
        circevent->setYSigma2(1);
        circevent->setID(0);
        outBottle.addEvent(circevent);

    }

    int bestxR, bestyR, bestrR;
    double bestScoreR = cObserverR->getObs(bestxR, bestyR, bestrR);

    if(bestScoreR > inlierThreshold) {

        //std::cout << bestx << " " << besty << " " << bestr << std::endl;
        event<ev::ClusterEventGauss> circevent = event<ev::ClusterEventGauss>(new ev::ClusterEventGauss());
        circevent->setStamp(q.back()->getStamp());
        circevent->setChannel(1);
        circevent->setXCog(bestxR);
        circevent->setYCog(bestyR);
        circevent->setXSigma2(bestrR);
        circevent->setYSigma2(1);
        circevent->setID(0);
        outBottle.addEvent(circevent);

    }

    //send on our event bottle
    if(strictness) outPort.writeStrict();
    else outPort.write();

    // ///////////////////
    // scope and debug images
    // ///////////////////

    //save the results
    if(!singleq && dumpOut.getOutputCount()) {

        double offsetts = yarp::os::Time::now() - tsoffset;

        yarp::os::Bottle &dumperL = dumpOut.prepare();
        dumperL.clear();
        dumperL.addDouble(offsetts);
        dumperL.addInt(q.back()->getStamp());
        dumperL.addInt(0);
        dumperL.addInt(bestxL);
        dumperL.addInt(bestyL);
        dumperL.addInt(bestrL);
        dumperL.addDouble(bestScoreL);
        dumpOut.setEnvelope(st);
        dumpOut.writeStrict();

        yarp::os::Bottle &dumperR = dumpOut.prepare();
        dumperR.clear();
        dumperR.addDouble(offsetts);
        dumperR.addInt(q.back()->getStamp());
        dumperR.addInt(1);
        dumperR.addInt(bestxR);
        dumperR.addInt(bestyR);
        dumperR.addInt(bestrR);
        dumperR.addDouble(bestScoreR);
        dumpOut.setEnvelope(st);
        dumpOut.writeStrict();
    }

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
        image = cObserverR->makeDebugImage();
        if(bestScoreR > inlierThreshold) {
            drawcircle(image, bestxR, bestyR, bestrR);
        }
        houghOut.setEnvelope(st);
        houghOut.write();
        //std::cout << "Processing Time" << timecounter << ", Hough response: " << cObserver->getObs(bestx, besty, bestr) << std::endl;
    }


}
