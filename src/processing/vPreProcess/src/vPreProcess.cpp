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
#include <opencv2/highgui.hpp>
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
    rf.setDefaultContext( "event-driven" );
    rf.setDefaultConfigFile( "vPreProcess.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vPreProcess module;
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return module.runModule(rf);
}

vPreProcess::vPreProcess(): name("/vPreProcess")
{
    v_total = 0;
    v_dropped = 0;

    outPortCamLeft.setWriteType(AE::tag);
    outPortCamRight.setWriteType(AE::tag);
    outPortCamStereo.setWriteType(AE::tag);
    outPortSkin.setWriteType(SkinEvent::tag);
    outPortSkinSamples.setWriteType(SkinSample::tag);
    out_port_aps_left.setWriteType(AE::tag);
    out_port_aps_right.setWriteType(AE::tag);
    out_port_imu_samples.setWriteType(IMUevent::tag);
    out_port_audio.setWriteType(AE::tag);
}


vPreProcess::~vPreProcess()
{
    inPort.close();
    outPortCamLeft.close();
    outPortCamRight.close();
    outPortCamStereo.close();
    outPortSkin.close();
    outPortSkinSamples.close();
    out_port_aps_left.close();
    out_port_aps_right.close();
    out_port_imu_samples.close();
    out_port_audio.close();
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
    undistort = rf.check("undistort") &&
            rf.check("undistort", Value(true)).asBool();
    flipx = rf.check("flipx") &&
            rf.check("flipx", Value(true)).asBool();
    flipy = rf.check("flipy") &&
            rf.check("flipy", Value(true)).asBool();
    precheck = rf.check("precheck") &&
            rf.check("precheck", Value(true)).asBool();
    split_stereo = rf.check("split_stereo") &&
            rf.check("split_stereo", Value(true)).asBool();
    split_polarities = rf.check("split_polarities") &&
                       rf.check("split_polarities", Value(false)).asBool();
    combined_stereo = rf.check("combined_stereo") &&
            rf.check("combined_stereo", Value(true)).asBool();
    use_local_stamp = rf.check("local_stamp") &&
            rf.check("local_stamp", Value(true)).asBool();

    if(!split_stereo) combined_stereo=true;

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
    if(undistort)
        yInfo() << "Applying camera undistortion";
    if(split_stereo)
        yInfo() << "Splitting into left/right streams";
    if(split_polarities)
        yInfo() << "Splitting into positive/negative streams";
    if(combined_stereo)
        yInfo() << "Producing combined stereo output";

    apply_filter = filter_spatial || filter_temporal;
    if(apply_filter)
    {
        filter_left.initialise(res.width, res.height);
        filter_right.initialise(res.width, res.height);
    }
    if(filter_spatial)
    {
        filter_left.use_spatial_filter(
                    rf.check("sf_time",
                             Value(0.05)).asDouble() * vtsHelper::vtsscaler,
                    rf.check("sf_size",
                             Value(1)).asInt());
        filter_right.use_spatial_filter(
                    rf.check("sf_time",
                             Value(0.05)).asDouble() * vtsHelper::vtsscaler,
                    rf.check("sf_size",
                             Value(1)).asInt());
    }

    if(filter_temporal)
    {
        filter_left.use_temporal_filter(
                    rf.check("tf_time",
                             Value(0.1)).asDouble() * vtsHelper::vtsscaler);
        filter_right.use_temporal_filter(
                    rf.check("tf_time",
                             Value(0.1)).asDouble() * vtsHelper::vtsscaler);
    }

    if(undistort) {

        if(calibrator.configure("camera", "stefi_calib.ini", 1))
            calibrator.showMapProjections(3.0);
        else
            yWarning() << "Could not correctly configure the cameras";
    }

    return Thread::start();

}

bool vPreProcess::threadInit()
{
    if(split_stereo) {
        if(split_polarities){
            if(!outPortCamLeft_pos.open(getName() + "/left_pos:o"))
                return false;
            if(!outPortCamRight_pos.open(getName() + "/right_pos:o"))
                return false;
            if(!outPortCamLeft_neg.open(getName() + "/left_neg:o"))
                return false;
            if(!outPortCamRight_neg.open(getName() + "/right_neg:o"))
                return false;
        } else {
            if (!outPortCamLeft.open(getName() + "/left:o"))
                return false;
            if (!outPortCamRight.open(getName() + "/right:o"))
                return false;
        }
        if(!out_port_aps_left.open(getName() + "/aps_left:o"))
            return false;
        if(!out_port_aps_right.open(getName() + "/aps_right:o"))
            return false;
    }
    if(combined_stereo) {
        if (split_polarities) {
            if (!outPortCamStereo_pos.open(getName() + "/AE_pos:o"))
                return false;
            if (!outPortCamStereo_neg.open(getName() + "/AE_neg:o"))
                return false;
        } else {
            if (!outPortCamStereo.open(getName() + "/AE:o"))
                return false;
        }
        if(!out_port_aps_stereo.open(getName() + "/APS:o"))
            return false;
    }

    if(!out_port_imu_samples.open(getName() + "/imu_samples:o"))
        return false;
    if(!out_port_audio.open(getName() + "/audio:o"))
        return false;
    if(!outPortSkin.open(getName() + "/skin:o"))
        return false;
    if(!outPortSkinSamples.open(getName() + "/skin_samples:o"))
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
    bool received_half_sample = false;
    int32_t salvage_sample[2] = {-1, 0};

    while(true) {

        double pyt = zynq_stamp.getTime();

        std::deque<AE> qleft_pos, qright_pos, qstereo_pos;
        std::deque<AE> qleft_neg, qright_neg, qstereo_neg;
        std::deque<AE> qleft, qright, qstereo;
        std::deque<int32_t> qskin;
        std::deque<int32_t> qskinsamples;
        std::deque<AE> qleft_aps, qright_aps, qstereo_aps;
        std::deque<int32_t> qimusamples;
        std::deque<int32_t> qaudio;

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

            if(IS_SKIN(*(qi+1))) { //IS_SKIN
                if(IS_SAMPLE(*(qi+1))) {
                    qskinsamples.push_back(*(qi++)); //TS
                    qskinsamples.push_back(*(qi++)); //VALUE/TAXEL
                } else {
                    qskin.push_back(*(qi++)); //TS
                    qskin.push_back(*(qi++)); //TAXEL
                }
            } else if(IS_IMUSAMPLE(*(qi+1))) { //IS_IMU
                qimusamples.push_back(*(qi++)); //TS
                qimusamples.push_back(*(qi++)); //VALUE
            } else if(IS_AUDIO(*(qi+1))) {
                qaudio.push_back(*(qi++)); //TS
                qaudio.push_back(*(qi++)); //EVENT
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
                if(apply_filter && !v.type) {
                    if(v.channel) {
                        if(!filter_right.check(v.x, v.y, v.polarity, v.stamp)) {
                            v_dropped++;
                            continue;
                        }
                    } else {
                        if(!filter_left.check(v.x, v.y, v.polarity, v.stamp)) {
                            v_dropped++;
                            continue;
                        }
                    }
                }

                //undistortion (including rectification)
                if(undistort) {
                    int x = v.x;
                    int y = v.y;
                    calibrator.sparseForwardTransform(v.channel, y, x);
                    v.x = x;
                    v.y = y;
                }
                if(split_stereo)
                {
                    if(v.channel)
                    {
                        if(v.type)
                            qright_aps.push_back(v);
                        else
                            if (split_polarities){
                                if (v.polarity){
                                    qright_pos.push_back(v);
                                } else {
                                    qright_neg.push_back(v);
                                }
                            } else {
                                qright.push_back(v);
                            }
                    }
                    else
                    {
                        if(v.type)
                            qleft_aps.push_back(v);
                        else
                        if (split_polarities){
                            if (v.polarity){
                                qleft_pos.push_back(v);
                            } else {
                                qleft_neg.push_back(v);
                            }
                        } else {
                            qleft.push_back(v);
                        }                    }
                }
                if(combined_stereo)
                {
                    if(v.type)
                        qstereo_aps.push_back(v);
                    else
                        qstereo.push_back(v);
                }
            }

        }

        if(qskinsamples.size()) { //if we have skin samples
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
        if(qleft_pos.size()) {
            outPortCamLeft_pos.write(qleft_pos, zynq_stamp);
        }
        if(qleft_neg.size()) {
            outPortCamLeft_neg.write(qleft_neg, zynq_stamp);
        }
        if(qright_pos.size()) {
            outPortCamRight_pos.write(qright_pos, zynq_stamp);
        }
        if(qright_neg.size()) {
            outPortCamRight_neg.write(qright_neg, zynq_stamp);
        }
        if(qstereo.size()) {
            outPortCamStereo.write(qstereo, zynq_stamp);
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
        if(qstereo_aps.size()) {
            out_port_aps_stereo.write(qstereo_aps, zynq_stamp);
        }
        if(qimusamples.size()) {
            out_port_imu_samples.write(qimusamples, zynq_stamp);
        }
        if(qaudio.size()) {
            out_port_audio.write(qaudio, zynq_stamp);
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
