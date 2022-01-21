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

#include <iomanip>
#include <stdio.h>
#include <numeric>
#include <algorithm>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "event-driven/core.h"
#include "event-driven/vis.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "raw_datatype.h"
#include "vision.h"
#include "imu.h"
#include "audio.h"
#include "skin.h"

using namespace ev;
using namespace yarp::os;
using std::string;

const std::string raw::tag = "AE";
class vPreProcess : public yarp::os::RFModule, public yarp::os::Thread
{
private:

    //output port for the vBottle with the new events computed by the module
    ev::BufferedPort<raw> input;
    yarp::os::BufferedPort< yarp::sig::Vector > rate_port;

    bool flag_vision;
    visionFunctions vision;

    bool flag_imu;
    imuFunctions imu;

    bool flag_skin;
    skinFunctions skin;

    bool flag_audio;
    audioFunctions audio;
    
    //pre-pre processing
    bool precheck;

    //output
    bool use_local_stamp;
    
    double rate_t{0.0};
    int rate_n{0};
    bool flag_stats{false};
    std::deque<double> plot_rates;
    void visualise_rate();

public:

    ~vPreProcess();

    //inherited functions
    virtual bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool interruptModule() override;
    void onStop() override;
    bool updateModule() override;
    void run() override;

};


vPreProcess::~vPreProcess() 
{
    input.close();
    vision.close();
    imu.close();
    skin.close();
    audio.close();
    rate_port.close();
}

bool vPreProcess::configure(yarp::os::ResourceFinder &rf) 
{
    if(rf.check("h") || rf.check("help")) {
        yInfo() << "--local_stamp <bool>: overwrite the packet stamp with one"
                   "immediately as the packet arrives";
        yInfo() << "--stats <bool>: visualise event-rate stats";
        yInfo() << "============";
        yInfo() << "--vision <bool>: open ports for vision";
        yInfo() << "--height <int>: image size";
        yInfo() << "--width <int>: image size";
        yInfo() << "--flipx <bool>: flip the image horizontally";
        yInfo() << "--flipy <bool>: flip the image vertically";
        yInfo() << "--polarities <bool>: separate streams based on polarity";
        yInfo() << "--combined_stereo <bool>: left/right in a single stream";
        yInfo() << "--corners <bool>: open a separate port for corners only."
                   "All events still exist in regular vision stream.";
        yInfo() << "--filter_s <double>: spatial filter time window (sec)";
        yInfo() << "--filter_t <double>: temporal filter time window (sec)";
        yInfo() << "--camera_calibration_file <path>: calibration file to use for undistort";
        yInfo() << "============";
        yInfo() << "--skin <bool>: open ports for skin";
        yInfo() << "============";
        yInfo() << "--imu <bool>: open ports for imu";
        yInfo() << "============";
        yInfo() << "--audio <bool>: open ports for audio";
        yInfo() << "============";
        return false;
    }

    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        yError() << "Could not find YARP";
        return false;
    }

    setName((rf.check("name", yarp::os::Value("/vPreProcess")).asString()).c_str());

    // global flags
    use_local_stamp = rf.check("local_stamp") &&
                      rf.check("local_stamp", Value(true)).asBool();
    flag_stats = rf.check("stats") &&
                 rf.check("stats", Value(true)).asBool();

    //vision flags
    flag_vision = rf.check("vision") &&
          rf.check("vision", Value(true)).asBool();
    unsigned int height = rf.check("height", Value(480)).asInt();
    unsigned int width = rf.check("width", Value(640)).asInt();
    bool undistort = rf.check("camera_calibration_file");
    bool flipx = rf.check("flipx") && rf.check("flipx", Value(true)).asBool();
    bool flipy = rf.check("flipy") && rf.check("flipy", Value(true)).asBool();
    double t_spatial = rf.check("filter_s", Value(0.0)).asFloat64();
    double t_temporal = rf.check("filter_t", Value(0.0)).asFloat64();
    bool output_polarities = rf.check("polarities") &&
                       rf.check("polarities", Value(true)).asBool();
    bool output_stereo = rf.check("combined_stereo") &&
                      rf.check("combined_stereo", Value(true)).asBool();

    bool output_corners = rf.check("corners") &&
              rf.check("corners", Value(true)).asBool();

    flag_imu = rf.check("imu") &&
               rf.check("imu", Value(true)).asBool();

    flag_skin = rf.check("skin") &&
                rf.check("skin", Value(true)).asBool();
        
    flag_audio = rf.check("audio") &&
                 rf.check("audio", Value(true)).asBool();

    // if(precheck)
    //     yInfo() << "Performing precheck for event corruption";

    flag_stats = true;
    flag_vision = true;

    if (flag_vision) {
        vision.init_splits(output_stereo, output_polarities, output_corners);
        vision.init_flips(flipx, flipy, {width, height});
        vision.init_filter(t_temporal, t_spatial);
        if(undistort)
            vision.init_undistort(rf.find("camera_calibration_file").asString());
        if(!vision.open(getName()))
            return false;
    }

    if (flag_skin)
        if(!skin.open(getName()))
            return false;

    if (flag_imu)
        if(!skin.open(getName()))
            return false;
    

    if (flag_audio)
        if(!skin.open(getName()))
            return false;
    

    if (!input.open(getName("/AE:i"))) {
        yError() << "Could not open" << getName("/AE:i");
        return false;
    }

    if(flag_stats) {
        cv::namedWindow("Event Rate", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Rate", 480, 360);
        cv::moveWindow("Event Rate", 580, 62);
    }

    return Thread::start();

}

double vPreProcess::getPeriod() {
    return 0.2;
}

void vPreProcess::visualise_rate()
{
    const static int s = 10; //scale
    const static int mr = 40; //max rate (whats the max rate we can see)
    cv::Mat canvas = cv::Mat::zeros(40*s, 40*s, CV_8UC1);
    for(auto i = 1; i < plot_rates.size(); i++) {
        cv::line(canvas, cv::Point((i-1)*s, plot_rates[i-1]*s), cv::Point(i*s, plot_rates[i]*s), CV_RGB(255, 255, 255), 2);
    }
    cv::line(canvas, cv::Point(0, 5*s), cv::Point(canvas.cols-1, 5*s), CV_RGB(128, 128, 128));
    cv::line(canvas, cv::Point(0, 10*s), cv::Point(canvas.cols-1, 10*s), CV_RGB(128, 128, 128));
    cv::line(canvas, cv::Point(0, 15*s), cv::Point(canvas.cols-1, 15*s), CV_RGB(128, 128, 128));
    cv::line(canvas, cv::Point(0, 30*s), cv::Point(canvas.cols-1, 30*s), CV_RGB(128, 128, 128));
    cv::flip(canvas, canvas, 0);
    cv::putText(canvas, "30M events/s", cv::Point(5, (mr -30)*s), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 1);
    cv::putText(canvas, "15M         ", cv::Point(5, (mr -15)*s), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 1);
    cv::putText(canvas, "10M         ", cv::Point(5, (mr -10)*s), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 1);
    cv::putText(canvas, " 5M         ", cv::Point(5, (mr - 5)*s), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), 1);
    cv::resize(canvas, canvas, cv::Size(480, 360));

    cv::imshow("Event Rate", canvas);
    cv::waitKey(1);
}

bool vPreProcess::updateModule() {

    int passed{0}, dropped{0};
    if(flag_vision)
        vision.stats(passed, dropped);

    if(passed) {
        auto pc = 100.0 * (double) passed / (double) (passed + dropped);
        int max_rate = this->getPeriod() * (double)rate_n / rate_t * 0.001;
        rate_n = 0; rate_t = 0.0;
        yInfo() << "Using" << (int)(passed*0.001) << "k/" << int((passed + dropped)*0.001)
                << "k(" << pc << "%)" << "of events."; 
                //<< "Maximum rate:" << max_rate << "k events / second.";
        auto &temp = rate_port.prepare();
        temp.resize(1);
        temp[0] = 0.000001 * ((double)passed + (double)dropped) / getPeriod();
        rate_port.write();
    }

    if(flag_stats) {
        plot_rates.push_back(0.000001 * ((double)passed + (double)dropped) / getPeriod());
        while (plot_rates.size() > 40) plot_rates.pop_front();
        visualise_rate();
    }

    //unprocessed data
    static int puqs = 0;
    int uqs = input.getPendingReads();
    if(uqs || puqs) {
        yInfo() << uqs << "unprocessed queues";
        puqs = uqs;
    }

    return Thread::isRunning();

}

void vPreProcess::run() {

    Stamp yarpstamp;
    Stamp localstamp;
    while (true) {

        ev::packet<raw> *q = input.read(true);
        if(!q) break;
        input.getEnvelope(yarpstamp);
        if (use_local_stamp) localstamp.update();
        else localstamp = yarpstamp;

        double tic = Time::now();
        for(auto &v : *q) {
            if(IS_SKIN(v.data)) { //IS_SKIN
                skin.process(&v);
            } else if(IS_IMU(v.data)) {
                imu.process((ev::IMUS *)&v);
            } else if(IS_AUDIO(v.data)) {
                audio.process((ev::earEvent *)&v);
            } else { //IS_VISION
                vision.process((ev::AE *)&v, yarpstamp.getTime());
            }
        }
        rate_t += Time::now() - tic;
        rate_n += q->size();

        vision.send(localstamp, q->duration());
        audio.send(localstamp, q->duration());
        imu.send(localstamp, q->duration());
        skin.send(localstamp, q->duration());

    }
}

bool vPreProcess::interruptModule() 
{
    return Thread::stop();
}

void vPreProcess::onStop() 
{
    input.close();
    vision.close();
    imu.close();
    skin.close();
    audio.close();
    rate_port.close();
}

int main(int argc, char *argv[]) {

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("event-driven");
    rf.setDefaultConfigFile("vPreProcess.ini");
    rf.configure(argc, argv);

    vPreProcess module;
    return module.runModule(rf);
}

// void vPreProcess::run() {
//     Stamp zynq_stamp;
//     Stamp local_stamp;

//     while (true) {

//         double pyt = zynq_stamp.getTime();

//         const ev::packet<int32_t> *q = input.read(true);
//         if(!q) break;
//         input.getEnvelope(zynq_stamp);

//         auto proc_start = Time::now();

//         delays.push_back((Time::now() - zynq_stamp.getTime()));
//         if(pyt) intervals.push_back(zynq_stamp.getTime() - pyt);

//         if(precheck) {
//             nm0 = zynq_stamp.getCount();
//             if(nm3 && nm0 - nm1 == 1 && nm1 - nm2 > 1 && nm1 - nm3 > 2) {
//                 yWarning() << "LOST" << nm1 - nm2 - 1 << "PACKETS ["
//                            << nm4 << nm3 << nm2 << nm1 << nm0 << "]"
//                            << q->size() << "packet size";
//             }
//             nm4 = nm3;
//             nm3 = nm2;
//             nm2 = nm1;
//             nm1 = nm0;
//         }

//         //unsigned int events_in_packet = 0;
//         const int32_t *qi = q->data();

//         while ((size_t) (qi - q->data()) < q->size()) {

//             if(IS_SKIN(*(qi + 1))) { //IS_SKIN
//                 if(IS_SAMPLE(*(qi + 1))) {
//                     qskinsamples.push_back(*(qi++)); //TS
//                     qskinsamples.push_back(*(qi++)); //VALUE/TAXEL
//                 } else {
//                     qskin.push_back(*(qi++)); //TS
//                     qskin.push_back(*(qi++)); //TAXEL
//                 }
//             } else if(IS_IMUSAMPLE(*(qi + 1))) { //IS_IMU
//                 qimusamples.push_back(*(qi++)); //TS
//                 qimusamples.push_back(*(qi++)); //VALUE
//             } else if(IS_AUDIO(*(qi + 1))) {
//                 qaudio.push_back(*(qi++)); //TS
//                 qaudio.push_back(*(qi++)); //EVENT
//             } else { // IS_VISION

//                 v.decode(qi);

//                 //precheck
//                 if(precheck && (v.x < 0 || v.x > resmod.width || v.y < 0 || v.y > resmod.height)) {
//                     yWarning() << "Event Corruption:" << v.getContent().toString();
//                     continue;
//                 }

//                 //flipx and flipy
//                 if(flipx) v.x = resmod.width - v.x;
//                 if(flipy) v.y = resmod.height - v.y;

//                 //salt and pepper filter (only to TD events)
//                 if(apply_filter && !v.type) {
//                     if(v.channel) {
//                         if(!filter_right.check(v.x, v.y, v.polarity, v.stamp)) {
//                             v_dropped++;
//                             continue;
//                         }
//                     } else {
//                         if(!filter_left.check(v.x, v.y, v.polarity, v.stamp)) {
//                             v_dropped++;
//                             continue;
//                         }
//                     }
//                 }

//                 //undistortion (including rectification)
//                 if(undistort) {
//                     int x = v.x;
//                     int y = v.y;
//                     calibrator.sparseForwardTransform(v.channel, y, x);
//                     v.x = x;
//                     v.y = y;
//                 }
//                 if(split_stereo) {
//                     if(v.channel) {
//                         if(v.type)
//                             qright_aps.push_back(v);
//                         else if(split_polarities) {
//                             if(v.polarity) {
//                                 qright_pos.push_back(v);
//                             } else {
//                                 qright_neg.push_back(v);
//                             }
//                         } else {
//                             qright.push_back(v);
//                         }
//                         if(corners && v.corner)
//                             qright_corners.push_back(v);
//                     } else {
//                         if(v.type)
//                             qleft_aps.push_back(v);
//                         else if(split_polarities) {
//                             if(v.polarity) {
//                                 qleft_pos.push_back(v);
//                             } else {
//                                 qleft_neg.push_back(v);
//                             }
//                         } else {
//                             qleft.push_back(v);
//                         }
//                         if(corners && v.corner)
//                             qleft_corners.push_back(v);
//                     }
//                 }
//                 if(combined_stereo) {
//                     if(v.type)
//                         qstereo_aps.push_back(v);
//                     else if(split_polarities) {
//                         if(v.polarity) {
//                             qstereo_pos.push_back(v);
//                         } else {
//                             qstereo_neg.push_back(v);
//                         }
//                     } else {
//                         qstereo.push_back(v);
//                     }
//                     if(corners && v.corner)
//                         qstereo_corners.push_back(v);
//                 }
//             }
//         }

//         if(qskinsamples.size()) { //if we have skin samples
//             //check if we need to fix the ordering
//             if(IS_SSV(qskinsamples[1])) { // missing address
//                 if(received_half_sample) { // but we have it from last bottle
//                     qskinsamples.push_front(salvage_sample[1]);
//                     qskinsamples.push_front(salvage_sample[0]);
//                 } else { // otherwise we are misaligned due to missing data
//                     qskinsamples.pop_front();
//                     qskinsamples.pop_front();
//                 }
//             }
//             received_half_sample = false; //either case the half sample is no longer valid

//             //check if we now have a cut event
//             int samples_overrun = qskinsamples.size() % packetSize(SkinSample::tag);
//             if(samples_overrun == 2) {
//                 salvage_sample[1] = qskinsamples.back();
//                 qskinsamples.pop_back();
//                 salvage_sample[0] = qskinsamples.back();
//                 qskinsamples.pop_back();
//                 received_half_sample = true;
//             } else if(samples_overrun) {
//                 yError() << "samples cut by " << samples_overrun;
//             }
//         }

//         v_total += qleft.size() + qright.size();

//         proc_times.push_back((v_total + v_dropped) / (Time::now() - proc_start));

//         if(use_local_stamp) {
//             local_stamp.update();
//             zynq_stamp = local_stamp;
//         }
//         if(qleft.size()) {
//             outPortCamLeft.write(qleft, zynq_stamp);
//         }
//         if(qright.size()) {
//             outPortCamRight.write(qright, zynq_stamp);
//         }
//         if(qleft_pos.size()) {
//             outPortCamLeft_pos.write(qleft_pos, zynq_stamp);
//         }
//         if(qleft_neg.size()) {
//             outPortCamLeft_neg.write(qleft_neg, zynq_stamp);
//         }
//         if(qright_pos.size()) {
//             outPortCamRight_pos.write(qright_pos, zynq_stamp);
//         }
//         if(qright_neg.size()) {
//             outPortCamRight_neg.write(qright_neg, zynq_stamp);
//         }
//         if(qstereo.size()) {
//             outPortCamStereo.write(qstereo, zynq_stamp);
//         }
//         if(qstereo_pos.size()) {
//             outPortCamStereo_pos.write(qstereo_pos, zynq_stamp);
//         }
//         if(qstereo_neg.size()) {
//             outPortCamStereo_neg.write(qstereo_neg, zynq_stamp);
//         }
//         if(qskin.size()) {
//             outPortSkin.write(qskin, zynq_stamp);
//         }
//         if(qskinsamples.size()) {
//             outPortSkinSamples.write(qskinsamples, zynq_stamp);
//         }
//         if(qleft_aps.size()) {
//             out_port_aps_left.write(qleft_aps, zynq_stamp);
//         }
//         if(qright_aps.size()) {
//             out_port_aps_right.write(qright_aps, zynq_stamp);
//         }
//         if(qstereo_aps.size()) {
//             out_port_aps_stereo.write(qstereo_aps, zynq_stamp);
//         }
//         if(qimusamples.size()) {
//             out_port_imu_samples.write(qimusamples, zynq_stamp);
//         }
//         if(qaudio.size()) {
//             out_port_audio.write(qaudio, zynq_stamp);
//         }
//         if(qleft_corners.size()) {
//             out_port_crn_left.write(qleft_corners, zynq_stamp);
//         }
//         if(qright_corners.size()) {
//             out_port_crn_right.write(qright_corners, zynq_stamp);
//         }
//         if(qstereo_corners.size()) {
//             out_port_crn_stereo.write(qstereo_corners, zynq_stamp);
//         }
//     }
// }


