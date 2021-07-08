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

using namespace ev;
using namespace yarp::os;
using std::string;

class visionFunctions 
{
private:

    //splitting options
    bool output_stereo{false};
    bool output_polarities{false};
    bool output_corners{false};
    
    //processing - flipping
    ev::resolution res{640, 480};
    bool flipx{false};
    bool flipy{false};

    //processing - undistort/rectify
    bool undistort{false};
    ev::vIPT calibrator;

    //processing - filter
    bool apply_filter{false};
    ev::vNoiseFilter filter_left;
    ev::vNoiseFilter filter_right;
    int v_total{0};
    int v_dropped{0};
    
    //ports and packets
    enum port_label { LEFT, RIGHT, LNEG, RNEG, LCOR, RCOR, STEREO};
    ev::BufferedPort<AE> ports[7];
    ev::packet<AE> *packets[7] = 
        {nullptr,nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

public:

    void stats(int &passed, int &dropped)
    {
        passed = v_total;
        dropped = v_dropped;
        v_total = 0;
        v_dropped = 0;
    }

    void init_flips(bool x, bool y, resolution r)
    {
        flipx = x;
        flipy = y;
        res = r;
    }

    void init_splits(bool stereo = false, bool polarities = false, bool corners = false) 
    {
        output_stereo = stereo;
        output_polarities = polarities;
        output_corners = corners;
    }

    void init_filter(double T_temporal = 0, double T_spatial = 0) 
    {
        if(T_temporal > 0.0 || T_spatial > 0.0) {
            filter_left.initialise(res.width, res.height);
            filter_right.initialise(res.width, res.height);
            apply_filter = true;
        }

        if (T_temporal > 0.0) {
            filter_left.use_temporal_filter(ev::secondsToTicks(T_temporal));
            filter_right.use_temporal_filter(ev::secondsToTicks(T_temporal));
        }

        if (T_spatial > 0.0) {
            filter_left.use_spatial_filter(ev::secondsToTicks(T_spatial));
            filter_right.use_spatial_filter(ev::secondsToTicks(T_spatial));
        }
    }

    void init_undistort(std::string calibration_file_path) 
    {
        if (calibrator.configure(calibration_file_path)) {
            calibrator.printValidCalibrationValues();
            undistort = true;
        } else {
            yError() << "Could not correctly configure the cameras";
        }
    }

    bool _openPort(const port_label label, const std::string name)
    {
        if (!ports[label].open(name)) {
            yError() << "Could not open" << name;
            return false;
        }
        packets[label] = &(ports[label].prepare());
        return true;
    }

    bool open(string mname)
    {
        if (output_stereo)
            if(!_openPort(STEREO, mname + "/stereo/AE:o"))
                return false;
        
        if (output_polarities) {
            if(!_openPort(LNEG, mname + "/left/neg/AE:o"))
                return false;
            if(!_openPort(RNEG, mname + "/right/neg/AE:o"))
                return false;
            if (!_openPort(LEFT, mname + "/left/pos/AE:o"))
                return false;
            if (!_openPort(RIGHT, mname + "/right/pos/AE:o"))
                return false;
        }

        if (output_corners) {
            if (!_openPort(LCOR, mname + "/left/corner/AE:o"))
                return false;
            if (!_openPort(RCOR, mname + "/right/corner/AE:o"))
                return false;
        }

        if (!output_polarities) {
            if (!_openPort(LEFT, mname + "/left/AE:o"))
                return false;
            if (!_openPort(RIGHT, mname + "/right/AE:o"))
                return false;
        }

        return true;
    }

    void process(AE *datum)
    {
        //flipping
        if (flipx) datum->x = res.width  - datum->x - 1;
        if (flipy) datum->y = res.height - datum->y - 1;

        //salt-n-pepper filter
        if (apply_filter) {
            if (datum->channel == ev::CAMERA_LEFT) {
                if (!filter_left.check(datum->x, datum->y, datum->p, datum->ts)) {
                    v_dropped++;
                    return;
                }
            } else {
                if (!filter_right.check(datum->x, datum->y, datum->p, datum->ts)) {
                    v_dropped++;
                    return;
                }
            }
        }

        v_total++;

        //undistortion and rectification
        if (undistort) {
            int y = datum->y; int x = datum->x;
            calibrator.sparseForwardTransform(datum->channel, y, x);
            datum->y = y; datum->x = x;
        }

        //output to stereo combined stream
        if (output_stereo) packets[STEREO]->push_back(*datum);

        //output to corners stream
        if (output_corners && datum->corner) {
            if(datum->channel == ev::CAMERA_LEFT)
                packets[LCOR]->push_back(*datum);
            else
                packets[RCOR]->push_back(*datum);
        }

        //output stereo split streams (splitting also by polarity if needed)
        if (output_polarities && datum->p == 0) {
            if (datum->channel == ev::CAMERA_LEFT)
                packets[LNEG]->push_back(*datum);
            else
                packets[RNEG]->push_back(*datum);
        } else {
            if (datum->channel == ev::CAMERA_LEFT)
                packets[LEFT]->push_back(*datum);
            else
                packets[RIGHT]->push_back(*datum);
        }
    }

    void send(Stamp stamp, double duration)
    {
        for(int pl = LEFT; pl <= STEREO; pl++) {
            if(packets[pl] && packets[pl]->size()) {
                packets[pl]->duration(duration);
                ports[pl].setEnvelope(stamp);
                ports[pl].write();
                packets[pl] = &(ports[pl].prepare());
            }
        }
    }

    void close()
    {
        for(int pl = LEFT; pl <= STEREO; pl++) {
            packets[pl] = nullptr;
            ports[pl].unprepare();
            ports[pl].close();
        }
    }

};

// class IMUfunctions 
// {
// private:

// public:



// };



class vPreProcess : public yarp::os::RFModule, public yarp::os::Thread
{
private:

    typedef struct raw {
        static const std::string tag;
        int32_t ts;
        int32_t data;
    } raw;
    
    //output port for the vBottle with the new events computed by the module
    ev::BufferedPort<raw> input;
    yarp::os::BufferedPort< yarp::sig::Vector > rate_port;

    bool flag_vision;
    visionFunctions vision;
    
    // ev::vWritePort outPortSkin;
    // ev::vWritePort outPortSkinSamples;
    // ev::vWritePort out_port_imu_samples;
    // ev::vWritePort out_port_audio;
    
    //pre-pre processing
    bool precheck;

    //output
    bool use_local_stamp;
    
    bool flag_stats{false};
    std::deque<double> plot_rates;
    void visualise_rate();

public:

    vPreProcess();
    ~vPreProcess();


    //inherited functions
    virtual bool configure(yarp::os::ResourceFinder &rf);
    double getPeriod();
    bool interruptModule();
    void onStop();
    bool threadInit();
    bool updateModule();
    void run();

};

vPreProcess::vPreProcess() {

    
    // outPortSkin.setWriteType(SkinEvent::tag);
    // outPortSkinSamples.setWriteType(SkinSample::tag);
    // out_port_imu_samples.setWriteType(IMUevent::tag);
    // out_port_audio.setWriteType(CochleaEvent::tag);
}


vPreProcess::~vPreProcess() {
    input.close();
    vision.close();

    // outPortSkin.close();
    // outPortSkinSamples.close();
    // out_port_imu_samples.close();
    // out_port_audio.close();

    rate_port.close();

}

bool vPreProcess::configure(yarp::os::ResourceFinder &rf) {

    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork(2)) {
        yError() << "Could not find YARP";
        return false;
    }

    setName((rf.check("name", yarp::os::Value("/vPreProcess")).asString()).c_str());

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
        return false;
    }

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




    

    // if(!split_stereo) combined_stereo = true;

    // if(precheck)
    //     yInfo() << "Performing precheck for event corruption";
    // if(flipx)
    //     yInfo() << "Flipping vision horizontally";
    // if(flipy)
    //     yInfo() << "Flipping vision vertically";
    // if(filter_spatial)
    //     yInfo() << "Applying spatial \"salt and pepper\" filter";
    // if(filter_temporal)
    //     yInfo() << "Applying temporal \"refractory\" filter";
    // if(undistort)
    //     yInfo() << "Applying camera undistortion";
    // if(split_stereo)
    //     yInfo() << "Splitting into left/right streams";
    // if(split_polarities)
    //     yInfo() << "Splitting into positive/negative streams";
    // if(combined_stereo)
    //     yInfo() << "Producing combined stereo output";
    flag_stats = true;
    flag_vision = true;
    if (flag_vision) {
        vision.init_splits(output_stereo, output_polarities, output_corners);
        vision.init_flips(flipx, flipy, {width, height});
        vision.init_filter(t_temporal, t_spatial);
        if(undistort)
            vision.init_undistort(rf.find("camera_calibration_file").asString());
        vision.open(getName());
    }

    if (!input.open(getName("/AE:i"))) {
        yError() << "Could not open" << getName("/AE:i");
        return false;
    }

    // apply_filter = filter_spatial || filter_temporal;
    // if(apply_filter) {
    //     filter_left.initialise(res.width, res.height);
    //     filter_right.initialise(res.width, res.height);
    // }
    // if(filter_spatial) {
    //     filter_left.use_spatial_filter(
    //             rf.check("sf_time",
    //                      Value(0.05)).asDouble() * vtsHelper::vtsscaler,
    //             rf.check("sf_size",
    //                      Value(1)).asInt());
    //     filter_right.use_spatial_filter(
    //             rf.check("sf_time",
    //                      Value(0.05)).asDouble() * vtsHelper::vtsscaler,
    //             rf.check("sf_size",
    //                      Value(1)).asInt());
    // }

    // if(filter_temporal) {
    //     filter_left.use_temporal_filter(
    //             rf.check("tf_time",
    //                      Value(0.1)).asDouble() * vtsHelper::vtsscaler);
    //     filter_right.use_temporal_filter(
    //             rf.check("tf_time",
    //                      Value(0.1)).asDouble() * vtsHelper::vtsscaler);
    // }

    // if(undistort) {

    //     std::string calib_file_path = rf.check("camera_calibration_file", Value("")).asString();
    //     if(calibrator.configure(calib_file_path, 1)) {
    //         calibrator.printValidCalibrationValues();
    //         //calibrator.showMapProjections(3.0);
    //     } else {
    //         yError() << "Could not correctly configure the cameras";
    //         return false;
    //     }
    // }

    if(flag_stats) {
        cv::namedWindow("Event Rate", cv::WINDOW_NORMAL);
        cv::resizeWindow("Event Rate", 480, 360);
        cv::moveWindow("Event Rate", 580, 62);
    }

    return Thread::start();

}

bool vPreProcess::threadInit() {
    return true;

    // if(!out_port_imu_samples.open(getName() + "/imu_samples:o"))
    //     return false;
    // if(!out_port_audio.open(getName() + "/audio:o"))
    //     return false;
    // if(!outPortSkin.open(getName() + "/skin:o"))
    //     return false;
    // if(!outPortSkinSamples.open(getName() + "/skin_samples:o"))
    //     return false;
    // if(!inPort.open(getName() + "/AE:i"))
    //     return false;
    // if(!rate_port.open(getName("/rate:o")))
    //     return false;

    // return true;
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
        auto max_rate = 0;
        yInfo() << "Using" << (int)(passed*0.001) << "k/" << int((passed + dropped)*0.001)
                << "k(" << pc << "%)" << "of events. Maximum rate:"
                << max_rate << "events / second.";
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

        for(auto &v : *q) {
            if(IS_SKIN(v.data)) { //IS_SKIN
                yWarning() << "skin not implemented";
            } else if(IS_IMUSAMPLE(v.data)) {
                yWarning() << "imu not implemented";
            } else if(IS_AUDIO(v.data)) {
                yWarning() << "audio not implemented";
            } else { //IS_VISION
                vision.process((AE *)&v);
            }
        }

        vision.send(localstamp, q->duration());
    }
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


bool vPreProcess::interruptModule() {
    return Thread::stop();
}

void vPreProcess::onStop() {
    input.close();
    vision.close();

    // outPortSkin.close();
    // outPortSkinSamples.close();
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

const std::string vPreProcess::raw::tag = "AE";
