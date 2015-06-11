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
#include <opencv2/opencv.hpp>

/**********************************************************/
bool vCircleModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vCircleFinder")
                                      ).asString();
    setName(moduleName.c_str());

    bool debugwindows = rf.check("debug",
                                 yarp::os::Value(false)).asBool();

    //sensory size
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();

    //activity memory
    double actDecay = rf.check("decay", yarp::os::Value(0.01)).asDouble();
    actDecay *= 1000000; //convert to us
    double actInject = rf.check("injection", yarp::os::Value(2)).asDouble();
    int actRadius = rf.check("radius", yarp::os::Value(0)).asInt();

    //observation parameters
    int obsRadius = rf.check("obsWindow", yarp::os::Value(20)).asDouble();
    int elimRegion = rf.check("elimRegion", yarp::os::Value(3)).asDouble();

    //filter parameters
    double procNoisePos = rf.check("procNoisePos",
                                   yarp::os::Value(1000)).asDouble();

    double procNoiseRad = rf.check("procNoiseRad",
                                   yarp::os::Value(1000)).asDouble();

    double measNoisePos = rf.check("measNoisePos",
                                   yarp::os::Value(10)).asDouble();

    double measNoiseRad = rf.check("measNoiseRad",
                                   yarp::os::Value(10)).asDouble();

    circleReader.setDebug(debugwindows);
    circleReader.resetObserverParams(width, height, actDecay, actInject,
                                     actRadius, obsRadius, elimRegion);
    circleReader.resetFilterParams(procNoisePos, procNoiseRad,
                                   measNoisePos, measNoiseRad);

    if(!circleReader.open(moduleName)) {
        std::cerr << "Could not open required ports" << std::endl;
        return false;
    }

    return true ;
}

/**********************************************************/
bool vCircleModule::interruptModule()
{
    circleReader.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCircleModule::close()
{
    circleReader.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCircleModule::updateModule()
{

    return true;
}

/**********************************************************/
double vCircleModule::getPeriod()
{
    return 0.3;

}
/**********************************************************/
vCircleReader::vCircleReader()
{

    //here we should initialise the module
//    yarp::sig::Matrix A(3, 3); A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1;
//    yarp::sig::Matrix H(3, 3); H = A;
//    yarp::sig::Matrix Q(3, 3); Q = 0;
//    Q(0, 0) = 0.1; Q(1, 1) = 0.1; Q(2, 2) = 1;
//    yarp::sig::Matrix R(3, 3); R = 0;
//    R(0, 0) = 32; R(1, 1) = 32; R(2, 2) = 16;

    yarp::sig::Matrix A(6, 6); A = 0;
    A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1;
    A(3, 3) = 1; A(4, 4) = 1; A(5, 5) = 1;
    //we need to update (0, 3), (1, 4) and (2, 5) based on delta t

    yarp::sig::Matrix H(6, 6); H = 0;
    H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;

    vPos = 0.1; vSiz = 1;
    yarp::sig::Matrix Q(6, 6); Q = 0;
    //we update Q depending on delta t

    yarp::sig::Matrix R(6, 6); R = 0;
    R(0, 0) = 32; R(1, 1) = 32; R(2, 2) = 16;


    filter = new iCub::ctrl::Kalman(A, H, Q, R);

    pTS = 0;
    periodstart = 0;//yarp::os::Time::now();
    filter_active = false;

    circleTracker = new vCircleTracker(0.1, 0.1, 32, 16);
    obscounter = 0;
    obstimer = 0;
    
}

/**********************************************************/
bool vCircleReader::open(const std::string &name)
{
    //and open the input port
    estimate = emorph::activityMat(128, 128, 200000, 0.05, 4);

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);
    filewriter.open("/home/aglover/temp.txt");
    return state1 && state2;
}

/**********************************************************/
void vCircleReader::close()
{
    //close ports
    outPort.close();
    if(filter) {
        delete filter;
        filter = 0;
    }
    if(circleTracker) {
        delete circleTracker;
        circleTracker = 0;
    }
    if(filewriter.is_open())
        filewriter.close();
    this->close();


    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void vCircleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    this->interrupt();


}

void vCircleReader::resetFilterParams(double pvp, double pvs, double mvp,
                                      double mvs)
{
    vPos = pvp;
    vSiz = pvs;

    yarp::sig::Matrix R(6, 6); R = 0;
    R(0, 0) = mvp; R(1, 1) = mvp; R(2, 2) = mvs;
    filter->set_R(R);
    if(circleTracker) delete circleTracker;
    circleTracker = new vCircleTracker(vPos, vSiz, mvp, mvs);
}

void vCircleReader::resetObserverParams(int width, int height, double aDec, double aInj,
                         int aRad, int oWin, int oTrim)
{
    //circleFinder = vCircleObserver();
}

/**********************************************************/
void vCircleReader::onRead(emorph::vBottle &bot)
{

    double tstart = yarp::os::Time::now();
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
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

    double count = 0;
    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        if(yarp::os::Time::now() - tstart > 0.0009) break;
        count++;

        emorph::AddressEvent *v = (*qi)->getAs<emorph::OpticalFlowEvent>();
        if(!v || v->getChannel()) continue;
        //if(!v->getPolarity()) continue;

        double cx, cy, cr;
        double e1 = circleFinder.flowcircle(cx, cy, cr);
        //double e1 = 12;





        if(cr < 5 || cr > 32) continue;
        if(e1 < circleFinder.minVsReq4RANSAC) continue;

        if(false && yarp::os::Time::now() - periodstart > 3) {
            circleFinder.flowView();
            cv::waitKey(20);
            periodstart = yarp::os::Time::now();
        }


        emorph::ClusterEventGauss circevent(*v);
        circevent.setChannel(v->getChannel());
        circevent.setXCog(cx);
        circevent.setYCog(cy);
        circevent.setXSigma2((int)cr);
        circevent.setYSigma2(/*circleFinder.inlierThreshold**/2);
        outBottle.addEvent(circevent);
        obscounter++;





    }

    if(count != q.size()) {
        //std::cout << "Processed " << count * 100.0 / q.size() << "%" << std::endl;
    }

    if(q.back()->getStamp() > obstimer + 100000 || q.back()->getStamp() < obstimer) {
        std::cout << obscounter << std::endl;
        obscounter = 0;
        obstimer = q.back()->getStamp();
    }


    //send on the processed events
    outPort.write();

    double tthread = yarp::os::Time::now() - tstart;
    //if(tthread > 0.001) {
        //std::cout << "On Read took too long " << tthread  << "ms" << std::endl;
    //}

}

//empty line to make gcc happy
