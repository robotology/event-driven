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
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

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
    ev::vWritePort eventPortOut;

    cv::Mat ref_values;
    cv::Mat prev_img;
    double prev_time;

    Stamp curr_stamp;

    struct
    {
        double C;
        double log_eps;
        double noise_variance;
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

        config_.log_eps = rf.check("log_eps", Value(0.001)).asDouble();
        config_.C = rf.check("C", Value(1e-6)).asDouble();
        config_.noise_variance = rf.check("noise_variance", Value(0.25)).asDouble();
        config_.use_log_image = rf.check("use_log_image", Value(true)).asBool();

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
        return 1;
    }

    bool updateModule()
    {
        return true;
    }

    deque<AE> processImage(const cv::Mat &img, double curr_time) {

        static constexpr double pixelscaler = 1.0 / 255.0;
        static Mat imgC1, img64, noise, diff64, abs64, abs8;

        //perform image pre-processing (grey, double, log)
        if(img.channels() > 1)
            cvtColor(img, imgC1, COLOR_RGB2GRAY);
        else
            imgC1 = img;
        imgC1.convertTo(img64, CV_64F, pixelscaler);

        if (config_.use_log_image)
            cv::log(config_.log_eps + img64, img64);


        //if first image we need to initialise based on the image size
        if (prev_img.empty()) {
            img64.copyTo(prev_img);
            ref_values = cv::Mat::zeros(prev_img.size(), CV_64F);
            noise = Mat::zeros(prev_img.size(), CV_64F);
            yInfo() << "Initialized event camera simulator with sensor size: " << prev_img.cols << "x" << prev_img.rows ;
            return {}; //do not produce events on the first image
        }

        //create the noise matrix
        double delta_t = curr_time - prev_time;
        prev_time = curr_time;
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
            v.x = event_list.at<cv::Point>(i).x;
            v.y = event_list.at<cv::Point>(i).y;
            v.stamp = curr_time * vtsHelper::vtsscaler; // To comply with ATIS timestamp
            v.polarity = (ref_values.at<double>(v.y, v.x) > 0.0) ? 0 : 1;
            events.push_back(v);

            ref_values.at<double>(v.y, v.x) = 0.0;
        }

        return events;
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
            deque<AE> events = processImage(yarp::cv::toCvMat(*yarpImage), curr_stamp.getTime());
            if(!events.empty())
                eventPortOut.write(events, curr_stamp);
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



