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
#include <numeric>
#include <algorithm>

using namespace ev;
using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
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
    vPreProcess module;
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return module.runModule(rf);
}

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
    out_port_aps_left.setWriteType(AE::tag);
    out_port_aps_right.setWriteType(AE::tag);
}


vPreProcess::~vPreProcess()
{
    inPort.close();
    outPortCamLeft.close();
    outPortCamRight.close();
    outPortSkin.close();
    outPortSkinSamples.close();
    out_port_aps_left.close();
    out_port_aps_right.close();
}

bool vPreProcess::configure(yarp::os::ResourceFinder &rf)
{
    setName((rf.check("name", yarp::os::Value("/vPreProcess")).asString()).c_str());

    res.height = rf.check("height", Value(240)).asInt();
    res.width = rf.check("width", Value(304)).asInt();

    bool filter_spatial = rf.check("filter_spatial") &&
            rf.check("filter_spatial", Value(true)).asBool();
    bool filter_temporal = rf.check("filter_temporal") &&
            rf.check("filter_temporal", Value(true)).asBool();
    rectify = rf.check("rectify") &&
            rf.check("rectify", Value(true)).asBool();
    undistort = rf.check("undistort") &&
            rf.check("undistort", Value(true)).asBool();
    truncate = rf.check("truncate") &&
            rf.check("truncate", Value(true)).asBool();
    flipx = rf.check("flipx") &&
            rf.check("flipx", Value(true)).asBool();
    flipy = rf.check("flipy") &&
            rf.check("flipy", Value(true)).asBool();
    precheck = rf.check("precheck") &&
            rf.check("precheck", Value(true)).asBool();
    split = rf.check("split") &&
            rf.check("split", Value(true)).asBool();
    use_local_stamp = rf.check("local_stamp") &&
            rf.check("local_stamp", Value(true)).asBool();
    if(precheck)
        yInfo() << "Performing precheck for event corruption";
    if(flipx)
        yInfo() << "Flipping vision horizontally";
    if(flipy)
        yInfo() << "Flipping vision vertically";
    if(filter_spatial)
        yInfo() << "Applying spatial \"salt and pepper\" filter";
    if(filter_temporal)
        yInfo() << "Applying temporal \"refractory\" filter";
    if(rectify)
        yInfo() << "Rectifying image pairs using extrinsic parameters";
    if(undistort && truncate)
        yInfo() << "Applying camera undistortion - truncating to sensor size";
    if(undistort && !truncate)
        yInfo() << "Applying camera undistortion - without truncation";
    if(split)
        yInfo() << "Splitting into left/right streams";

    apply_filter = filter_spatial || filter_temporal;
    if(apply_filter)
    {
        thefilter.initialise(res.width, res.height);
    }
    if(filter_spatial)
    {
        thefilter.use_spatial_filter(
                    rf.check("sf_time",
                             Value(0.05)).asDouble() * vtsHelper::vtsscaler,
                    rf.check("sf_size",
                             Value(1)).asInt());
    }

    if(filter_temporal)
    {
        thefilter.use_temporal_filter(
                    rf.check("tf_time",
                             Value(0.1)).asDouble() * vtsHelper::vtsscaler);
    }

    if(undistort) {
        ResourceFinder calibfinder;
        calibfinder.setVerbose();
        calibfinder.setDefaultContext(rf.check("calibContext", Value("cameraCalib")).asString().c_str());
        calibfinder.setDefaultConfigFile(rf.check("calibFile", Value("iCubEyes-ATIS.ini")).asString().c_str());
        calibfinder.configure(0, 0);

        Bottle &leftParams = calibfinder.findGroup("CAMERA_CALIBRATION_LEFT");
        Bottle &rightParams = calibfinder.findGroup("CAMERA_CALIBRATION_RIGHT");
        Bottle &stereoParams = calibfinder.findGroup("STEREO_DISPARITY");
        if(leftParams.isNull() || rightParams.isNull()) {
            yError() << "Could not load intrinsic camera parameters";
            return false;
        }
        if (rectify && stereoParams.isNull()) {
            yError() << "Could not load extrinsic camera parameters";
            return false;
        }

        std::cout << leftParams.toString() << std::endl;
        std::cout << rightParams.toString() << std::endl;
        std::cout << stereoParams.toString() << std::endl;
        initUndistortion(leftParams, rightParams, stereoParams, truncate);
    }

    return Thread::start();

}

bool vPreProcess::threadInit()
{
    if(split) {
        if(!outPortCamLeft.open(getName() + "/left:o"))
            return false;
        if(!outPortCamRight.open(getName() + "/right:o"))
            return false;
        if(!out_port_aps_left.open(getName() + "/aps_left:o"))
            return false;
        if(!out_port_aps_right.open(getName() + "/aps_right:o"))
            return false;
    } else {
        if(!outPortCamLeft.open(getName() + "/AE:o"))
            return false;
        if(!out_port_aps_left.open(getName() + "/APS:o"))
            return false;
    }
    if(!outPortSkin.open(getName() + "/skin:o"))
        return false;
    if(!outPortSkinSamples.open(getName() + "/skinsamples:o"))
        return false;
    if(!inPort.open(getName() + "/AE:i"))
        return false;
    return true;
}

double vPreProcess::getPeriod()
{
    return 2.0;
}

bool vPreProcess::updateModule()
{
    if(v_total) {
        auto pc = 100.0 * (double)v_total / (double)(v_total + v_dropped);
        auto max_rate =
                std::accumulate(proc_times.begin(), proc_times.end(), 0.0) /
                proc_times.size();
        proc_times.clear();
        yInfo() << "Using" << v_total << "/" << (v_total + v_dropped)
                << "(" << pc << "%)" << "of events. Maximum rate:"
                << max_rate << "events / second.";
    }
    v_total = 0;
    v_dropped = 0;

    //unprocessed data
    static int puqs = 0;
    int uqs = inPort.queryunprocessed();
    if(uqs || puqs) {
        yInfo() << uqs << "unprocessed queues";
        puqs = uqs;
    }

    return Thread::isRunning();

    //delays
    if(!delays.size())
        return Thread::isRunning();

    auto bounds  = std::minmax_element(delays.begin(), delays.end());
    auto min_d = *bounds.first * 1000;
    auto max_d = *bounds.second * 1000;

    auto mean_d = 1000 *
            std::accumulate(delays.begin(), delays.end(), 0.0) / delays.size();
    delays.clear();

    auto meanr = vtsHelper::vtsscaler *
            std::accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
    rates.clear();

    auto meani = 1000 *
            std::accumulate(intervals.begin(), intervals.end(), 0.0) / intervals.size();
    intervals.clear();

    //yInfo() << mind << meand << maxd << " : min | mean | max";
    yWarning() << std::fixed << min_d << " " << mean_d << " "
              << max_d << " " << meanr << " " << meani;

    return Thread::isRunning();
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
        std::deque<AE> qleft_aps, qright_aps;

        const std::vector<int32_t> *q = inPort.read(zynq_stamp);
        if(!q) break;

        auto proc_start = Time::now();

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

                //salt and pepper filter (only to TD events)
                if(apply_filter && !v.type && !thefilter.check(v.x, v.y, v.polarity, v.channel, v.stamp)) {
                    v_dropped++;
                    continue;
                }

                //undistortion (including rectification)
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

                }

                if(split && v.channel)
                {
                    if(v.type)
                        qright_aps.push_back(v);
                    else
                        qright.push_back(v);
                } else {
                    if(v.type)
                        qleft_aps.push_back(v);
                    else
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

        proc_times.push_back((v_total + v_dropped) / (Time::now() - proc_start));

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
        if(qleft_aps.size()) {
            out_port_aps_left.write(qleft_aps, zynq_stamp);
        }
        if(qright_aps.size()) {
            out_port_aps_right.write(qright_aps, zynq_stamp);
        }
    }
}

void vPreProcess::initUndistortion(const yarp::os::Bottle &left,
                                   const yarp::os::Bottle &right,
                                   const yarp::os::Bottle &stereo, bool truncate)
{
    this->truncate = truncate;
    const yarp::os::Bottle *coeffs[3] = { &left, &right, &stereo};
    cv::Mat *maps[2] = {&leftMap, &rightMap};
    cv::Mat cameraMatrix[2];
    cv::Mat distCoeffs[2];
    cv::Mat rectRot[2];
    cv::Size s(res.height, res.width);
    cv::Mat Proj[2];

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

        cv::Mat defCamMat = cv::getDefaultNewCameraMatrix(cameraMatrix[i], s, true);
        Proj[i] = defCamMat;
    }

    if(rectify)
    {
        //Loading extrinsic stereo parameters
        yarp::os::Bottle *HN = coeffs[2]->find("HN").asList();
        if(HN == nullptr || HN->size() != 16)
            yError() << "Rototranslation matrix HN is absent or without required number of values: 16)";
        else
        {
            std::cout<<"After extracting list from bottle value HN: "<<(HN->toString())<<std::endl;

            cv::Mat R(3, 3, CV_64FC1); //Rotation matrix between stereo cameras
            cv::Mat T(3, 1, CV_64FC1); //Translation vector of right wrt left camera center
            for (int row=0; row<3; row++)
            {
                for(int col=0; col<3; col++)
                {
                    R.at<double>(row, col) = HN->get(row*4 + col).asDouble();
                }
                T.at<double>(row) = HN->get(row*4+3).asDouble();
            }
            std::cout<<"R and T values stored properly; R:"<<R<<"T: "<<T<<std::endl;

            cv::Mat R_left(3, 3, CV_64FC1);
            cv::Mat R_right(3, 3, CV_64FC1);
            cv::Mat P_left(3, 4, CV_64FC1);
            cv::Mat P_right(3, 4, CV_64FC1);
            cv::Mat Q(4, 4, CV_64FC1);
            //Computing homographies for left and right image
            cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                    s, R, T, R_left, R_right, P_left, P_right, Q, CV_CALIB_ZERO_DISPARITY);
            rectRot[0] = R_left.clone();
            rectRot[1] = R_right.clone();
            Proj[0] = P_left.clone();
            Proj[1] = P_right.clone();

        }
    }

    for(int i=0; i<2; i++) {
        cv::Mat allpoints(res.height * res.width, 1, CV_32FC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                allpoints.at<cv::Vec2f>(y * res.width + x) = cv::Vec2f(x, y);
            }
        }

        cv::Mat mappoints(res.height * res.width, 1, CV_32FC2);

        cv::undistortPoints(allpoints, mappoints, cameraMatrix[i], distCoeffs[i],
                            rectRot[i], Proj[i]);
        *(maps[i]) = cv::Mat(res.height, res.width, CV_32SC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                maps[i]->at<cv::Vec2i>(y, x) =
                        mappoints.at<cv::Vec2f>(y * res.width + x);
            }
        }
    }
}

bool vPreProcess::interruptModule()
{
    return Thread::stop();
}

void vPreProcess::onStop()
{
    inPort.close();
    outPortCamLeft.close();
    outPortCamRight.close();
    outPortSkin.close();
    outPortSkinSamples.close();
}


