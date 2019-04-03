/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vPreProcess.h"
#include <iomanip>
#include <stdio.h>

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find YARP";
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vPreProcess.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vPreProcessModule preProcessModule;
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return preProcessModule.runModule(rf);
}

/******************************************************************************/
bool vPreProcessModule::configure(yarp::os::ResourceFinder &rf)
{
    bool pepper = rf.check("pepper") &&
            rf.check("pepper", yarp::os::Value(true)).asBool();
    bool undistort = rf.check("undistort") &&
            rf.check("undistort", yarp::os::Value(true)).asBool();
    bool truncate = rf.check("truncate") &&
            rf.check("truncate", yarp::os::Value(true)).asBool();
    bool flipx = rf.check("flipx") &&
            rf.check("flipx", yarp::os::Value(true)).asBool();
    bool flipy = rf.check("flipy") &&
            rf.check("flipy", yarp::os::Value(true)).asBool();
    bool precheck = rf.check("precheck") &&
            rf.check("precheck", yarp::os::Value(true)).asBool();
    bool split = rf.check("split") &&
            rf.check("split", yarp::os::Value(true)).asBool();
    bool local_stamp = rf.check("local_stamp") &&
            rf.check("local_stamp", yarp::os::Value(true)).asBool();

    if(precheck)
        yInfo() << "Performing precheck for event corruption";
    if(flipx)
        yInfo() << "Flipping vision horizontally";
    if(flipy)
        yInfo() << "Flipping vision vertically";
    if(pepper)
        yInfo() << "Applying salt and pepper filter";
    if(undistort && truncate)
        yInfo() << "Applying camera undistortion - truncating to sensor size";
    if(undistort && !truncate)
        yInfo() << "Applying camera undistortion - without truncation";
    if(split)
        yInfo() << "Splitting into left/right streams";

    eventManager.initBasic(rf.check("name", yarp::os::Value("/vPreProcess")).asString(),
                           rf.check("height", yarp::os::Value(240)).asInt(),
                           rf.check("width", yarp::os::Value(304)).asInt(),
                           precheck, flipx, flipy, pepper, undistort, split, local_stamp);

    if(pepper) {
        eventManager.initPepper(rf.check("spatialSize", yarp::os::Value(1)).asDouble(),
                                rf.check("temporalSize", yarp::os::Value(0.1)).asDouble() * vtsHelper::vtsscaler);
    }

    if(undistort) {
        yarp::os::ResourceFinder calibfinder;
        calibfinder.setVerbose();
        calibfinder.setDefaultContext(rf.check("calibContext", yarp::os::Value("SumanContext")).asString().c_str());
        calibfinder.setDefaultConfigFile(rf.check("calibFile", yarp::os::Value("iCubEyes-ATIS.ini")).asString().c_str());
        calibfinder.configure(0, 0);

        yarp::os::Bottle &leftParams = calibfinder.findGroup("CAMERA_CALIBRATION_LEFT");
        yarp::os::Bottle &rightParams = calibfinder.findGroup("CAMERA_CALIBRATION_RIGHT");
        yarp::os::Bottle &stereoParams = calibfinder.findGroup("STEREO_DISPARITY");
        if(leftParams.isNull() || rightParams.isNull() || stereoParams.isNull()) {
            yError() << "Could not load camera parameters";
            return false;
        }
        std::cout << leftParams.toString() << std::endl;
        std::cout << rightParams.toString() << std::endl;
        std::cout << stereoParams.toString() << std::endl;
        eventManager.initUndistortion(leftParams, rightParams, stereoParams, truncate);
    }

    return eventManager.start();

}

bool vPreProcessModule::close()
{
    eventManager.stop();
    return yarp::os::RFModule::close();
}

bool vPreProcessModule::updateModule()
{
    eventManager.printFilterStats();
    //unprocessed data
    static int puqs = 0;
    int uqs = this->eventManager.queryUnprocessed();
    if(uqs || puqs) {
        yInfo() << uqs << "unprocessed queues";
        puqs = uqs;
    }

    return true;

    //delays
    std::deque<double> dcopy = eventManager.getDelays();

    if(!dcopy.size())
        return true;

    double mind = 1e6;
    double maxd = -1e6;
    double meand = 0;
    for(size_t i = 0; i < dcopy.size(); i++) {
        mind = std::min(mind, dcopy[i]);
        maxd = std::max(maxd, dcopy[i]);
        meand += dcopy[i];
    }
    meand /= dcopy.size();
    meand *= 1000;
    mind *= 1000;
    maxd *= 1000;

    double meanr = 0;
    std::deque<double> rcopy = eventManager.getRates();
    for(size_t i = 0; i < rcopy.size(); i++) {
        meanr += rcopy[i];
    }
    meanr /= rcopy.size();
    meanr *= vtsHelper::vtsscaler;

    double meani = 0;
    std::deque<double> icopy = eventManager.getIntervals();
    for(size_t i = 0; i < icopy.size(); i++) {
        meani += icopy[i];
    }
    meani /= icopy.size();
    meani *= 1000;

    //yInfo() << mind << meand << maxd << " : min | mean | max";
    std::cout << std::fixed << mind << " " << meand << " " << maxd << " " << meanr << " " << meani << std::endl;

    return true;
}

double vPreProcessModule::getPeriod()
{
    return 2.0;
}
/******************************************************************************/
vPreProcess::vPreProcess(): name("/vPreProcess")
{
    leftMap.deallocate();
    rightMap.deallocate();
    v_total = 0;
    v_dropped = 0;

    outPortCamLeft.setWriteType(AE::tag);
    outPortCamRight.setWriteType(AE::tag);
    outPortSkin.setWriteType(SkinEvent::tag);
    outPortSkinSamples.setWriteType(SkinSample::tag);
}


vPreProcess::~vPreProcess()
{
    inPort.close();
    outPortCamLeft.close();
    outPortCamRight.close();
    outPortSkin.close();
    outPortSkinSamples.close();
}

void vPreProcess::initBasic(std::string name, int height, int width,
                            bool precheck, bool flipx, bool flipy,
                            bool pepper, bool undistort, bool split,
                            bool local_stamp)
{

    this->name = name;
    res.height = height;
    res.width = width;
    this->precheck = precheck;
    this->flipx = flipx;
    this->flipy = flipy;
    this->pepper = pepper;
    this->undistort = undistort;
    this->split = split;
    this->use_local_stamp = local_stamp;

}

void vPreProcess::initPepper(int spatialSize, int temporalSize)
{
    thefilter.initialise(res.width, res.height, temporalSize, spatialSize);
}

void vPreProcess::initUndistortion(const yarp::os::Bottle &left,
                               const yarp::os::Bottle &right,
                               const yarp::os::Bottle &stereo, bool truncate)
{
    bool rectify = true;

    this->truncate = truncate;
    const yarp::os::Bottle *coeffs[3] = { &left, &right, &stereo};
    cv::Mat *maps[2] = {&leftMap, &rightMap};
    cv::Mat cameraMatrix[2];
    cv::Mat distCoeffs[2];

    //create camera and distortion matrices
    for(int i = 0; i < 2; i++) {

        double scaley = res.height / (double)(coeffs[i]->find("h").asInt());
        double scalex = res.width  / (double)(coeffs[i]->find("w").asInt());

        cameraMatrix[i] = cv::Mat(3, 3, CV_64FC1);
        cameraMatrix[i].setTo(0);
        cameraMatrix[i].at<double>(0, 0) = coeffs[i]->find("fx").asDouble()*scalex;
        cameraMatrix[i].at<double>(1, 1) = coeffs[i]->find("fy").asDouble()*scaley;
        cameraMatrix[i].at<double>(2, 2) = 1.0;
        cameraMatrix[i].at<double>(0, 2) = coeffs[i]->find("cx").asDouble()*scalex;
        cameraMatrix[i].at<double>(1, 2) = coeffs[i]->find("cy").asDouble()*scaley;

        distCoeffs[i] = cv::Mat(4, 1, CV_64FC1);
        distCoeffs[i].at<double>(0, 0) = coeffs[i]->find("k1").asDouble();
        distCoeffs[i].at<double>(0, 1) = coeffs[i]->find("k2").asDouble();
        distCoeffs[i].at<double>(0, 2) = coeffs[i]->find("p1").asDouble();
        distCoeffs[i].at<double>(0, 3) = coeffs[i]->find("p2").asDouble();
      }
std::cout<<"Before loading extrinsic parameters"<<std::endl;
      //Loading extrinsic stereo parameters
      // yarp::sig::VectorOf<double> RT;
      yarp::os::Bottle *a = coeffs[2]->find("HN").asList();
      std::cout<<"After extracting list from bottle value HN: "<<(a->toString())<<std::endl;

      // a->write(RT);
      // std::cout<<"Stored in Vector RT "<<RT.toString()<<std::endl;

      cv::Mat R(3, 3, CV_64FC1);
      cv::Mat T(3, 1, CV_64FC1);
      for (int row=0; row<3; row++)
      {
        for(int col=0; col<3; col++)
        {
          R.at<double>(row, col) = a->get(row*4 + col).asDouble();
        }
        T.at<double>(row) = a->get(row*4+3).asDouble();
      }
      std::cout<<"R and T values stored properly; R:"<<R<<"T: "<<T<<std::endl;

      cv::Mat R_left(3, 3, CV_64FC1);
      cv::Mat R_right(3, 3, CV_64FC1);
      cv::Mat P_left(3, 4, CV_64FC1);
      cv::Mat P_right(3, 4, CV_64FC1);
      cv::Mat Q(4, 4, CV_64FC1);
      //Computing homographies for left and right image
      cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
        cv::Size(res.height, res.width), R, T, R_left, R_right, P_left, P_right, Q);
      cv::Mat Rot[2] = {R_left, R_right};


      for(int i=0; i<2; i++) {
        cv::Mat allpoints(res.height * res.width, 1, CV_32FC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                allpoints.at<cv::Vec2f>(y * res.width + x) = cv::Vec2f(x, y);
            }
        }

        cv::Mat mappoints(res.height * res.width, 1, CV_32FC2);
        cv::Size s(res.height, res.width);
        cv::Mat defCamMat = cv::getDefaultNewCameraMatrix(cameraMatrix[0], s, true);
std::cout<<"homographies "<<i<<": "<<Rot[i]<<std::endl;
        cv::Mat H;
        if(rectify)
          H = Rot[i].clone();
        cv::undistortPoints(allpoints, mappoints, cameraMatrix[i], distCoeffs[i],
                            H, defCamMat);

        *(maps[i]) = cv::Mat(res.height, res.width, CV_32SC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                maps[i]->at<cv::Vec2i>(y, x) =
                        mappoints.at<cv::Vec2f>(y * res.width + x);
            }
        }
    }
}

int vPreProcess::queryUnprocessed()
{
    return inPort.queryunprocessed();
}

void vPreProcess::printFilterStats()
{
    if(v_total) {
        double pc = 100.0 * (double)v_total / (double)(v_total + v_dropped);
        yInfo() << "Using" << v_total << "/" << (v_total + v_dropped)
                << "(" << pc << "%)" << "of events.";
    }
    v_total = 0;
    v_dropped = 0;

}

std::deque<double> vPreProcess::getDelays()
{
    std::deque<double> dcopy = delays;
    delays.clear();
    return dcopy;
}

std::deque<double> vPreProcess::getRates()
{
    std::deque<double> rcopy = rates;
    rates.clear();
    return rcopy;
}

std::deque<double> vPreProcess::getIntervals()
{
    std::deque<double> icopy = intervals;
    intervals.clear();
    return icopy;
}

void vPreProcess::run()
{
    Stamp zynq_stamp;
    Stamp local_stamp;

    resolution resmod = res;
    resmod.height -= 1;
    resmod.width -= 1;
    int nm0 = 0, nm1 = 0, nm2 = 0, nm3 = 0, nm4 = 0;
    AE v;
    SkinEvent se;
    SkinSample ss;
    bool received_half_sample = false;
    int32_t salvage_sample[2] = {-1, 0};

    while(true) {

        double pyt = zynq_stamp.getTime();

        std::deque<AE> qleft, qright;
        std::deque<int32_t> qskin;
        std::deque<int32_t> qskinsamples;
        const std::vector<int32_t> *q = inPort.read(zynq_stamp);
        if(!q) break;

        delays.push_back((Time::now() - zynq_stamp.getTime()));
        if(pyt) intervals.push_back(zynq_stamp.getTime() - pyt);

        if(precheck) {
            nm0 = zynq_stamp.getCount();
            if(nm3 && nm0 - nm1 == 1 && nm1 - nm2 > 1 && nm1 - nm3 > 2) {
                yWarning() << "LOST" << nm1-nm2-1 << "PACKETS ["
                           << nm4 << nm3 << nm2 << nm1 << nm0 << "]"
                           << q->size() << "packet size";
            }
            nm4 = nm3;
            nm3 = nm2;
            nm2 = nm1;
            nm1 = nm0;
        }

        //unsigned int events_in_packet = 0;
        const int32_t *qi = q->data();

        while ((size_t)(qi - q->data()) < q->size()) {

            if(IS_SKIN(*(qi+1))) {
                if(IS_SAMPLE(*(qi+1))) {
                    qskinsamples.push_back(*(qi++)); //TS
                    qskinsamples.push_back(*(qi++)); //VALUE/TAXEL
                } else {
                    qskin.push_back(*(qi++)); //TS
                    qskin.push_back(*(qi++)); //TAXEL
                }
            } else { // IS_VISION

                v.decode(qi);

                //precheck
                if(precheck && (v.x < 0 || v.x > resmod.width || v.y < 0 || v.y > resmod.height)) {
                    yWarning() << "Event Corruption:" << v.getContent().toString();
                    continue;
                }

                //flipx and flipy
                if(flipx) v.x = resmod.width - v.x;
                if(flipy) v.y = resmod.height - v.y;

                //salt and pepper filter
                if(pepper && !thefilter.check(v.x, v.y, v.polarity, v.channel, v.stamp)) {
                    v_dropped++;
                    continue;
                }

                //undistortion
                if(undistort) {
                    cv::Vec2i mapPix;
                    if(v.getChannel() == 0)
                        mapPix = leftMap.at<cv::Vec2i>(v.y, v.x);
                    else
                        mapPix = rightMap.at<cv::Vec2i>(v.y, v.x);

                    //truncate to sensor bounds after mapping?
                    if(truncate && (mapPix[0] < 0 ||
                                    mapPix[0] > resmod.width ||
                                    mapPix[1] < 0 ||
                                    mapPix[1] > resmod.height)) {
                        continue;
                    }

                    v.x = mapPix[0];
                    v.y = mapPix[1];
                    std::cout.precision(30);
                    std::cout<<v.channel<<mapPix<<"timestamp:"<<pyt<<std::endl;

                }

                if(split && v.channel)
            {
                    qright.push_back(v);
             }   else {
                    qleft.push_back(v);
}
            }

        }

        if(qskinsamples.size() > 2) { //if we have skin samples
            //check if we need to fix the ordering
            if(IS_SSV(qskinsamples[1])) { // missing address
                    if(received_half_sample) { // but we have it from last bottle
                        qskinsamples.push_front(salvage_sample[1]);
                        qskinsamples.push_front(salvage_sample[0]);
                    } else { // otherwise we are misaligned due to missing data
                        qskinsamples.pop_front();
                        qskinsamples.pop_front();
                    }
            }
            received_half_sample = false; //either case the half sample is no longer valid

            //check if we now have a cut event
            int samples_overrun = qskinsamples.size() % packetSize(SkinSample::tag);
            if(samples_overrun == 2) {
                salvage_sample[1] = qskinsamples.back();
                qskinsamples.pop_back();
                salvage_sample[0] = qskinsamples.back();
                qskinsamples.pop_back();
                received_half_sample = true;
            } else if(samples_overrun) {
                yError() << "samples cut by " << samples_overrun;
            }
        }

        v_total += qleft.size() + qright.size();

        if(use_local_stamp) {
            local_stamp.update();
            zynq_stamp = local_stamp;
        }

        if(qleft.size()) {
            outPortCamLeft.write(qleft, zynq_stamp);
        }
        if(qright.size()) {
            outPortCamRight.write(qright, zynq_stamp);
        }
        if(qskin.size()) {
            outPortSkin.write(qskin, zynq_stamp);
        }
        if(qskinsamples.size()) {
            outPortSkinSamples.write(qskinsamples, zynq_stamp);
        }
    }

}

void vPreProcess::onStop()
{
    inPort.close();
    outPortCamLeft.close();
    outPortCamRight.close();
    outPortSkin.close();
    outPortSkinSamples.close();
}

bool vPreProcess::threadInit()
{
    if(split) {
      std::cout<<"Reaching here"<<std::endl;
        if(!outPortCamLeft.open(name + "/left:o")){
          std::cout<<"Can't open port:"<<name<<std::endl;
            return false;
          }
        if(!outPortCamRight.open(name + "/right:o"))
            return false;
    } else {
        if(!outPortCamLeft.open(name + "/left:o"))
            return false;
    }
    if(!outPortSkin.open(name + "/skin:o"))
        return false;
    if(!outPortSkinSamples.open(name + "/skinsamples:o"))
        return false;
    if(!inPort.open(name + "/AE:i"))
        return false;
    return true;
}
