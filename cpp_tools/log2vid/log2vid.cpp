#include <yarp/os/all.h>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <opencv2/opencv.hpp>
#include <event-driven/core.h>
#include <event-driven/algs.h>

//using yarp::os::ResourceFinder;
namespace fs = std::filesystem;

void helpfunction() 
{
    yInfo() << "USAGE:";
    yInfo() << "--file <string> logfile path";
    yInfo() << "--out <string> output video [~/Downloads/events.mp4]";
    yInfo() << "--fps <int> frames per second of output video [240]";
    yInfo() << "--rate <double> speed-up/slow-down factor [1.0]";
    yInfo() << "--height <int> video height [720]";
    yInfo() << "--width <int> video width [1280]";
    yInfo() << "--vis <bool> show conversion process [false]";
    yInfo() << "METHOD: --iso";
    yInfo() << "--parameters";
    yInfo() << "METHOD: --tw";
    yInfo() << "--window <double> seconds of window length [0.01]";
    yInfo() << "METHOD: --scarf";
    yInfo() << "--block_size <int> size of a array in pixels [14]";
    yInfo() << "--alpha <double> ratio of events in block";
    yInfo() << "--C <double> "
}

// cv::Mat createImageTW(ev::offlineLoader<ev::AE> &loader, double toc, double win_duration)
// {
//     static std::deque<ev::AE> buffer;
//     for(auto &v : loader)
//         buffer.push_back(v);

//     while
//             img.at<cv::Vec3b>(v.y, v.x) = {255, 255, 255};


// }

int main(int argc, char* argv[])
{

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    if(rf.check("help") || rf.check("h")) {
        helpfunction();
        return 0;
    }

    if(!rf.check("file")) {
        yError() << "Please provide path to .log containing events";
        helpfunction();
        return -1;
    }
    std::string file_path = rf.find("file").asString();

    int fps = rf.check("fps", yarp::os::Value(240)).asInt32();
    double rate = rf.check("rate", yarp::os::Value(1.0)).asFloat64();
    double period = 1.0/fps;
    cv::Size res = {rf.check("width", yarp::os::Value(1280)).asInt32(), 
                    rf.check("height", yarp::os::Value(720)).asInt32()};
    bool vis = rf.check("vis") &&
               rf.check("vis", yarp::os::Value(true)).asBool();
    double duration = rf.check("window", yarp::os::Value(0.01)).asFloat64();

    ev::offlineLoader<ev::AE> loader;
    yInfo() << "Loading log file ... ";
    if(!loader.load(file_path)) {
        yError() << "Could not open log file";
        return false;
    } else {
        yInfo() << loader.getinfo();
    }

    std::string defaultpath = std::string(std::getenv("HOME")) + "/Downloads/events.mp4";
    cv::VideoWriter dw;
    dw.open(defaultpath,
            cv::VideoWriter::fourcc('a','v','c','1'),
            fps, res, true);

    double virtual_timer = period;
    loader.synchroniseRealtimeRead(0.0);

    if(rf.find("tw").asBool()) 
    {

        while(loader.windowedReadTill(virtual_timer, duration)) {
            cv::Mat img = cv::Mat::zeros(res, CV_8UC3);

            for(auto &v : loader)
                img.at<cv::Vec3b>(v.y, v.x) = {255, 255, 255};
            if(vis) {
                cv::imshow("events-log2vid", img);
                cv::waitKey(1);
            }
            dw << img;
            virtual_timer += period;
        }

    } 
    // else if(rf.find("scarf").asBool()) 
    // {
    //     ev::SCARF scarf;
    //     scarf.initialise(res, )

    //     while(loader.incrementReadTill(virtual_timer)) {
    //         cv::Mat img = cv::Mat::zeros(res, CV_8UC3);

    //         for(auto &v : loader)
    //             img.at<cv::Vec3b>(v.y, v.x) = {255, 255, 255};
    //         if(vis) {
    //             cv::imshow("events-log2vid", img);
    //             cv::waitKey(1);
    //         }
    //         dw << img;
    //         virtual_timer += period;
    //     }

    // }

    dw.release();

    return 0;

}