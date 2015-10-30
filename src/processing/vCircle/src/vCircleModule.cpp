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

    bool strictness = rf.check("strict", yarp::os::Value(false)).asBool();

    bool parallel = rf.check("parallel", yarp::os::Value(false)).asBool();

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();


    //observation parameters
    double inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(50)).asDouble() / 100.0;

    std::string qType = rf.check("qType",
                                 yarp::os::Value("Lifetime")).asString();

    std::string houghType = rf.check("houghType",
                              yarp::os::Value(true)).asString();

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

    bool flowhough = true;
    if(houghType == "full") flowhough = false;
    //circleReader.houghFinder.qType = qType;
    //circleReader.houghFinder.useFlow = flowhough;

    circleReader.cObserver =
            new vCircleMultiSize(qType, 2000, 10, 35, flowhough, parallel, width, height);

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
    std::cout << "vCirle spent " << this->timecounter
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

/******************************************************************************/
void vCircleReader::onRead(emorph::vBottle &inBot)
{
    
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = inBot;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);
    if(!pstamp.isValid()) pstamp = st;

    //create event queue
    emorph::vQueue q = inBot.get<emorph::AddressEvent>();
    q.wrapSort();

    if(!q.size()) {
        if(strictness) outPort.writeStrict();
        else outPort.write();
        return;
    }

    double t1 = yarp::os::Time::now();
    cObserver->addQueue(q);
    timecounter += yarp::os::Time::now() - t1;

    int bestx, besty, bestr;
    double bestts = unwrap(q.back()->getStamp());

    if(cObserver->getObs(bestx, besty, bestr) > inlierThreshold) {

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

    //send on our event bottle
    if(strictness) outPort.writeStrict();
    else outPort.write();

    double dstamp = st.getTime() - pstamp.getTime();
    if(houghOut.getOutputCount() && (dstamp > 0.03333 || dstamp < 0)) {
        pstamp = st;
        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = houghOut.prepare();
        image = cObserver->makeDebugImage();
        houghOut.setEnvelope(st);
        houghOut.write();
        std::cout << timecounter << std::endl;
    }


}