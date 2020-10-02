/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author:massimiliano.iacono@iit.it
 *          arren.glover@iit.it
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


/**
 * The EventSimulator takes as input a sequence of stamped images,
 * assumed to be sampled at a "sufficiently high" framerate,
 * and simulates the principle of operation of an idea event camera
 * with a constant contrast threshold C.
 * Pixel-wise intensity values are linearly interpolated in time.
 *
 * The pixel-wise voltages are reset with the values from the first image
 * which is passed to the simulator.
 */

#include <yarp/os/all.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>

#include <random>
#include <execution>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <filesystem>
#include <event-driven/vPort.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace cv;
using namespace ev;
using std::deque;

class EsimModule : public yarp::os::RFModule,
                   public yarp::os::Thread
{

private:

    BufferedPort< ImageOf<PixelRgb> > imgPortIn;
    RpcServer rpcPort;
    ev::vWritePort eventPortOut;
    yarp::os::Port debugPort;
    std::vector<std::deque<ev::AE>> queues;
    std::vector<double> timestamps;
    cv::Mat ref_values;
    cv::Mat prev_img;
    double prev_time;
    double first_time;

    bool recording;
    Stamp curr_stamp;

    struct
    {
        double C;
        double refractory_period;
        double log_eps;
        double noise_variance;
        double ts_noise_range;
        bool use_log_image;
    } config_;

public:

    bool configure(yarp::os::ResourceFinder &rf)
    {

        std::string moduleName = rf.check("name", Value("/esim-yarp"), "module name (string)").asString();
        setName(moduleName.c_str());

        yarp::os::Network yarp;
        if (!yarp.checkNetwork(2.0))
        {
            yError()<<"YARP server not available!";
            return false;
        }

        if(!imgPortIn.open(getName("/image:i"))) {
            yError() << "Could not open" << getName("/image:i");
            return false;
        }

        if(!eventPortOut.open(getName("/AE:o"))) {
            yError() << "Could not open" << getName("/AE:o");
            return false;
        }

        if(!debugPort.open(getName("/debug"))) {
            yError() << "Could not open" << getName("/debug");
            return false;
        }

        if(!rpcPort.open(getName("/rpc"))) {
            yError() << "Could not open" << getName("/rpc");
            return false;
        }

        config_.log_eps = rf.check("log_eps", Value(0.001)).asDouble();
        config_.C = rf.check("C", Value(1e-6)).asDouble();
        config_.noise_variance = rf.check("noise_variance", Value(0.25)).asDouble();
        config_.ts_noise_range = rf.check("ts_noise_range", Value(0.04)).asDouble();
        config_.refractory_period = rf.check("refractory_period", Value(10e-6)).asDouble();
        config_.use_log_image = rf.check("use_log_image", Value(true)).asBool();
        curr_stamp.update();
        first_time = curr_stamp.getTime();
        recording = false;

        return Thread::start();
    }

    bool close()
    {
        imgPortIn.close();
        eventPortOut.close();
        return true;
    }

    double getPeriod()
    {
        return 0.1;
    }

    bool updateModule() {
        Bottle cmd_bottle;
        Bottle response;
        rpcPort.read(cmd_bottle, true);
        std::string cmd = cmd_bottle.get(0).toString();

        static std::string savePath;
        std::ostringstream response_ss;

        if (cmd == "record") {
            std::string path = cmd_bottle.get(1).toString();

            if (std::filesystem::is_directory(path)) {
                response_ss << "Recording data in " << path;
                savePath = path;
                recording = true;
            } else if (std::filesystem::is_regular_file(path)) {
                response_ss << path << " is a file. Please provide a directory.";
            } else if (!std::filesystem::exists(path)){
                response_ss << "Cannot access " << path << ". Please provide a valid path to store the data";
            }

        } else if (cmd == "stop") {
            if (!recording){
                response_ss << "Please start a recording before trying to stop it.";
            } else {
                response_ss << "Stopping recording.";
                recording = false;
                saveDataYarpFormat(savePath);
                saveDataTxtFormat(savePath);
                queues.clear();
                timestamps.clear();
            }
        }

        response.addString(response_ss.str());

        rpcPort.reply(response);

        return true;
    }

    deque<AE> processImage(const cv::Mat &img, double curr_time) {

        static constexpr double pixelscaler = 1.0 / 255.0;
        static Mat imgC1, img64, noise, diff64, abs64, abs8, last_timestamp;
        static double ts_noise;
        //perform image pre-processing (grey, double, log)
        if(img.channels() > 1)
            cvtColor(img, imgC1, COLOR_RGB2GRAY);
        else
            imgC1 = img;
        imgC1.convertTo(img64, CV_64F, pixelscaler);

        if (config_.use_log_image)
            cv::log(config_.log_eps + img64, img64);


        //create the noise matrix
        double delta_t = curr_time - prev_time;
        prev_time = curr_time;

        //if first image we need to initialise based on the image size
        if (prev_img.empty()) {
            img64.copyTo(prev_img);
            ref_values = cv::Mat::zeros(prev_img.size(), CV_64F);
            last_timestamp = cv::Mat::zeros(prev_img.size(), CV_64F);
            noise = Mat::zeros(prev_img.size(), CV_64F);
            yInfo() << "Initialized event camera simulator with sensor size: " << prev_img.cols << "x" << prev_img.rows ;
            return {}; //do not produce events on the first image
        }

        cv::randn(noise, 0, config_.noise_variance*delta_t);

        //create the image difference and move current image to previous image
        diff64 = img64 - prev_img;
        img64.copyTo(prev_img);

        //update the reference values from diff and noise
        ref_values += (diff64 + noise);

        // For each pixel, check if new events need to be generated since the last image sample
        abs64 = cv::abs(ref_values);
        cv::threshold(abs64, abs64, config_.C, 1.0, CV_THRESH_TOZERO);
        abs64.convertTo(abs8, CV_8U);

        // create events for each pixel that exceeds the threshold
        cv::Mat event_list;
        cv::findNonZero(abs8, event_list);

        static ev::AddressEvent v;
        std::deque<ev::AddressEvent> events;
        for (size_t i = 0; i < event_list.total(); i++) {
            int x = event_list.at<cv::Point>(i).x;
            int y = event_list.at<cv::Point>(i).y;

            double ref_val = ref_values.at<double>(y, x);

            int num_events = std::min(delta_t / config_.refractory_period, abs(ref_val) / config_.C);
            if (num_events <= 0 ) continue;

            double time_step = delta_t / num_events;

            for (int j = 1; j <= num_events; ++j) {
                v.x = x;
                v.y = y;
                ts_noise = double(random()) * config_.ts_noise_range / RAND_MAX - config_.ts_noise_range / 2;
                double timestamp = (curr_time - delta_t + j * time_step + ts_noise);
                timestamp = min(timestamp, curr_time);
                timestamp = max(timestamp, curr_time - delta_t);
                unsigned int timestamp_int = timestamp * vtsHelper::vtsscaler ;
                v.stamp = timestamp_int;
                v.polarity = (ref_val > 0.0) ? 0 : 1;
                events.push_back(v);
            }
            last_timestamp.at<double>(y, x) = curr_time;

            ref_values.at<double>(y, x) = 0.0;
        }

        std::sort(std::execution::par_unseq, events.begin(), events.end(),
                  [](const ev::AddressEvent &a, const ev::AddressEvent &b) {
                      return a.stamp < b.stamp;
                  });

        return events;
    }

    void saveDataYarpFormat(std::string path) {
        std::filesystem::path dir (path);
        std::filesystem::path file ("data.log");
        std::filesystem::path completePath = dir / file;
        yInfo() << "Saving data to " << completePath;
        std::ofstream dataFile (completePath);
        if (dataFile.is_open())
        {
            for (int j = 0; j < queues.size(); ++j) {
                dataFile << std::fixed << j << " " << timestamps.at(j) << " AE " << "(";
                for (int k = 0; k < queues.at(j).size(); ++k) {
                    AddressEvent v = queues.at(j).at(k);
                    dataFile << std::fixed << v.stamp << " " << v._coded_data;
                    if (k == queues.at(j).size() - 1) break;
                    dataFile << " ";
                }
                dataFile << ")" << std::endl;
            }
            dataFile.close();
            yInfo() << "Successfully saved data to " << completePath;
        } else {
            std::cout << "Unable to open file";
        }
    }

    void saveDataTxtFormat(std::string path) {
        std::filesystem::path dir (path);
        std::filesystem::path file ("data.txt");
        std::filesystem::path completePath = dir / file;
        yInfo() << "Saving data to " << completePath;
        std::ofstream dataFile (completePath);
        if (dataFile.is_open())
        {
            for (int j = 0; j < queues.size(); ++j) {
                for (int k = 0; k < queues.at(j).size(); ++k) {
                    AddressEvent v = queues.at(j).at(k);
                    dataFile << v.x << " " << v.y << " " << v.polarity << " " << v.stamp << std::endl;
                }
            }
            dataFile.close();
            yInfo() << "Successfully saved data to " << completePath;
        } else {
            std::cout << "Unable to open file";
        }
    }

    void run() {

        while (!Thread::isStopping()) {

            //read new image (blocking)
            yarp::sig::ImageOf<yarp::sig::PixelRgb>* yarpImage = imgPortIn.read();
            if(!yarpImage)
                return;

            //update timing information based on image when images are available
            //this could use the imgPortIn.getEnvelope() if it is valid
            curr_stamp.update();

            //produce and write events
            double currTime = curr_stamp.getTime() - first_time;
            deque<AE> events = processImage(yarp::cv::toCvMat(*yarpImage), currTime);

            if(!events.empty()){
                eventPortOut.write(events, curr_stamp);
                if(recording) {
                    queues.push_back(events);
                    timestamps.push_back(currTime);
                }
            }

        }
    }

};


int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile( "esim-yarp.ini" );
    rf.setDefaultContext("event-driven");
    rf.configure(argc,argv);

    EsimModule module;
    return module.runModule(rf);
}



