/*
 *   Copyright (C) 2021 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <yarp/os/all.h>
#include "event-driven/core.h"
#include "event-driven/vis.h"
#include <thread>

using namespace ev;
using namespace yarp::os;

class calibration_module : public RFModule {

private:
    ev::window<ev::AE> input;
    std::thread black_thread;
    cv::Mat black_img;
    cv::Size img_size;

public:

    bool configure(yarp::os::ResourceFinder& rf) override
    {

        if(rf.check("h") || rf.check("help")) {
            yInfo() << "Calibration of event-camera";
            yInfo() << "--name <str>\t: internal port name prefix";
            yInfo() << "--folder <str>\t: folder path storing calibi.ini and images";
            yInfo() << "--generate_images <bool>\t: save more images for calibration";
            yInfo() << "--parameters_mono <bool>\t: generate calibration from images";
            return false;
        }

        if(!yarp::os::Network::checkNetwork(2.0)) {
            std::cout << "Could not connect to YARP" << std::endl;
            return false;
        }

        setName((rf.check("name", Value("/calibrate_eventcamera")).asString()).c_str());

        if(!input.open(getName("/AE:i"))) {
            yError() << "could not open input port";
            return false;
        }

        black_thread = std::thread([this]{eventsToBlack();});
        
        return true;
    }

    double getPeriod() override
    {
        return 0.1; //period of synchronous thread
    }

    void eventsToBlack()
    {
        while(!isStopping()) {
            if (black_img.empty())
                black_img = cv::Mat(img_size, CV_8UC3);
            else
                black_img = ev::black;

            ev::info stats = input.readSlidingWinT(0.033, false);

            for (auto& v : input)
                black_img.at<cv::Vec3b>(v.y, v.x) = white;
        }
    }

    bool interruptModule() override
    {
        //when stop(), isStopping()=true and interruptModule() is called
        black_thread.join();
        input.stop();
        return true;
    }

    //synchronous thread
    bool updateModule() override
    {
        cv::imshow("", black_img);
        return true;
    }
};

int main(int argc, char * argv[])
{

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.configure( argc, argv );

    /* create the module */
    calibration_module instance;
    return instance.runModule(rf);
}
