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

    circleReader.open(moduleName);

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
    periodstart = yarp::os::Time::now();
    filter_active = false;

    circleTracker = new vCircleTracker(0.1, 0.1, 32, 16);
    
}

/**********************************************************/
bool vCircleReader::open(const std::string &name)
{
    //and open the input port
    estimate = emorph::activityMat(128, 128, 200000, 0.05, 4);

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);
    filewriter.open("/home/aglover/temp.txt");
    return true;
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
    double ransact = 0;
    double t0;
    //create event queue
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot;
    //bot.getAll(q);

//    if(debugFlag)
//    {
//        circleFinder.stepbystep = true;
//        for(qi = q.begin(); qi != q.end(); qi++)
//        {
//            emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
//            if(!v) continue;
//            if(v->getChannel()) continue;
//            bottleImage.at<char>(v->getX(), v->getY()) = 255;
//        }

//    }

    //if(q.size()) {
    //    double cx, cy, cr;
    //    circleFinder.gradient(cx, cy, cr);
    //}



    for(qi = q.begin(); qi != q.end(); qi++)
    {      

        //filewriter << (*qi)->getStamp() << std::endl;
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        unsigned long int ts = unwrap(v->getStamp());
        if(v->getChannel()) continue;
        if(v->getPolarity()) continue;
        double cx, cy, cr;
        //circleTracker->addEvent(*v, 0.1);

        //continue;

        double dt;
        //dt = ((double)ts - pTS) * emorph::vtsHelper::tstosecs();
        //circleTracker->predict(dt);



        circleFinder.addEvent(**qi);
        t0 = yarp::os::Time::now();
        //circleFinder.localCircleEstimate(*v, cx, cy, cr, false);
        double e1;
        e1 = circleFinder.gradient(cx, cy, cr);
        //cv::waitKey(10);
        //e1 = circleFinder.RANSAC(cx, cy, cr);
        //e1 = circleFinder.oneShotObserve(cx, cy, cr);
        ransact += yarp::os::Time::now() - t0;
        if(e1 < circleFinder.minVsReq4RANSAC) continue;

        //cv::waitKey(1000);
        //continue;

        //std::cout << cr << std::endl;

        emorph::ClusterEventGauss circevent(*v);
        circevent.setChannel(v->getChannel());
        circevent.setXCog(cx);
        circevent.setYCog(cy);
        circevent.setXSigma2((int)cr);
        circevent.setYSigma2(circleFinder.inlierThreshold*2);
        outBottle.addEvent(circevent);

        if(true && yarp::os::Time::now() - periodstart > 5) {
            //circleFinder.viewEvents();
            //circleFinder.gradientView();
            std::cout << e1 << std::endl;
            circleFinder.gradientView2();
            cv::waitKey(20);
            periodstart = yarp::os::Time::now();
        }


        continue;

        //continue;
        //if(!circleFinder.localCircleEstimate(*v, cx, cy, cr, debugFlag)) continue;



        if(!filter_active) {
            yarp::sig::Vector x0(6, 0.0); x0[0] = cx; x0[1] = cy; x0[2] = cr;
            yarp::sig::Matrix P0 = filter->get_R();
            P0(3, 3) = P0(0, 0); P0(4, 4) = P0(1, 1); P0(5, 5) = P0(2, 2);
            filter->init(x0, P0);
            filter_active = true;
            pTS = ts;

            //circleTracker->init(cx, cy, cr);

        } else {

            //double dt = ((double)ts - pTS) / 1000000.0;

            circleTracker->predict(dt);
            //if(circleTracker->Pzgd(cx, cy, cr) < 0.01) continue;
            circleTracker->correct(cx, cy, cr, dt);



            yarp::sig::Vector z(6, 0.0); z[0]=cx; z[1]=cy; z[2]=cr;
            yarp::sig::Matrix A = filter->get_A();
            A(0, 3) = dt; A(1, 4) = dt; A(2, 5) = dt;
            filter->set_A(A);

            yarp::sig::Matrix Q = filter->get_Q();
            Q(3, 3) = vPos*dt; Q(4, 4) = vPos*dt; Q(5, 5) = vSiz*dt;
            filter->set_Q(Q);

            filter->filt(z);




            pTS = ts;
        }

        //for our estimator
        //emorph::AddressEvent tevent(*v);
        //tevent.setStamp(v->getStamp());
        //tevent.setX(cx);
        //tevent.setY(cy);
        //estimate.addEvent(tevent);
    }

//    if(debugFlag) {
//        t0 = yarp::os::Time::now();
//        cv::flip(bottleImage, bottleImage, 0);
//        cv::resize(bottleImage, bottleImage, bottleImage.size()*2);
//        cv::imshow("All Bottle", bottleImage);
//        cv::waitKey(1);
//        showing += yarp::os::Time::now() - t0;
//    }

//    if(false && circleTracker->active) {
//        yarp::sig::Vector x = circleTracker->filter->get_x();
//        yarp::sig::Matrix P = circleTracker->filter->get_P();

//        //std::cout << "x:" << std::endl << x.toString() << std::endl;
//        //std::cout << "P:" << std::endl << P.toString() << std::endl;


//        cv::Mat filterview(512, 512, CV_8UC3); filterview.setTo(0);

//        //validation gate
//        //cv::Mat valMat = filterview(cv::Rect())
//        std::ostringstream ss; ss << circleTracker->filter->get_ValidationGate();
//        cv::putText(filterview, ss.str(), cv::Point(10, 118*4), 0, 1, CV_RGB(0, 255, 0));

//        //estimated state
//        if(x[2] > 0) {
//            cv::circle(filterview, cv::Point(x[1]*4, 511-x[0]*4), x[2]*4,
//                    CV_RGB(255, 255, 255), 3);
//        }

//        //x-y standard deviation
//        if(P(0, 0) > 0) {
//            cv::circle(filterview, cv::Point(x[1]*4, 511-x[0]*4),
//                    (P(0, 0)+P(1, 1))*2, CV_RGB(255, 255, 0));
//        }

//        //radius standard deviation
//        //upperbound
//        double rv = std::fabs(P(2, 2));
//        if(x[2] + rv > 0) {
//            cv::circle(filterview, cv::Point(x[1]*4, 511-x[0]*4),
//                    (x[2]+rv)*4, CV_RGB(0, 255, 255));
//        }
//        //lowerbound
//        if(x[2] - rv > 0) {
//            cv::circle(filterview, cv::Point(x[1]*4, 511-x[0]*4),
//                    (x[2]-rv)*4, CV_RGB(0, 255, 255));
//        }

//        //x-y velocity
//        cv::Point pc(x[1]*4, 511-x[0]*4); cv::Point pv = pc + cv::Point(x[4], -x[3]);
//        cv::line(filterview, pc, pv, CV_RGB(255, 0, 0));


//        cv::imshow("Filter", filterview);
//        cv::waitKey(1);


//    }

//    if(debugFlag) {

//        //visualising the estimator
//        double mact = 0; int mx, my;
//        cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
//        for(int x = 0; x < 128; x++) {
//            for(int y = 0; y < 128; y++) {
//                double a = estimate.queryActivity(x, y);
//                if(mact < a) {
//                    mact = a;
//                    mx = x;
//                    my = y;
//                }
//                mact = std::max(a, mact);
//                image2.at<float>(127 - x, y) = a;
//            }
//        }
//        image2 = image2 * (1/mact);
//        cv::circle(image2, cv::Point(my, 127 - mx), 12, CV_RGB(255, 255, 255));
//        cv::Mat image(512, 512, CV_32F);
//        cv::resize(image2, image, image.size());

//        cv::imshow("Local Estimate", image);
//        cv::waitKey(1);
//    }





    //send on the processed events
    outPort.write();

    double tthread = yarp::os::Time::now() - tstart;
    //if(ransact > 0.001) {
    //    std::cout << "On Read took " << tthread / 0.001 << "x too long (" <<
    //                 100 * ransact / tthread <<  "% from RANSAC)" << std::endl;
    //}

}

//empty line to make gcc happy
