#include <yarp/os/all.h>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <opencv2/opencv.hpp>
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <event-driven/vis.h>

//using yarp::os::ResourceFinder;
namespace fs = std::filesystem;
using yarp::os::Value;

void helpfunction() 
{
    yInfo() << "USAGE:";
    yInfo() << "--file <string> logfile path";
    yInfo() << "--out <string> output video [~/Downloads/events.mp4]";
    yInfo() << "--timestamps <string> input timestamps filepath [optional]";
    yInfo() << "--fps <int> frames per second of output video [240]";
    yInfo() << "--rate <double> speed-up/slow-down factor [1.0]";
    yInfo() << "--height <int> video height [720]";
    yInfo() << "--width <int> video width [1280]";
    yInfo() << "--vis <bool> show conversion process [false]";
    yInfo() << "METHOD: iso [default]";
    yInfo() << "--window <double> seconds of window length [0.5]";
    yInfo() << "METHOD: --tw";
    yInfo() << "--window <double> seconds of window length [0.01]";
    yInfo() << "METHOD: --scarf";
    yInfo() << "--block_size <int> array dimension [14]";
    yInfo() << "--alpha <double> events accumulation factor [1.0]";
    yInfo() << "--C <double> intensity [0.3]";
    yInfo() << "METHOD: --eros";
    yInfo() << "--block_size <int> array dimension [7]";
    yInfo() << "--alpha <double> events decay factor [0.3]";
}

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

    int fps = rf.check("fps", Value(240)).asInt32();
    double rate = rf.check("rate", Value(1.0)).asFloat64();
    double period = 1.0/fps;
    cv::Size res = {rf.check("width", Value(1280)).asInt32(), 
                    rf.check("height", Value(720)).asInt32()};
    bool vis = rf.check("vis") &&
               rf.check("vis", Value(true)).asBool();
    std::ifstream stampfile;
    if(rf.check("timestamps")) 
    {
        stampfile.open(rf.find("timestamps").asString());
        if(!stampfile.is_open()) {
            yError() << "--timestamps provided but the file could not be opened" << rf.find("stamps").asString();
        } else {
            yInfo() << "Using timestamp file for frame timing";
        }
    }    

    ev::offlineLoader<ev::AE> loader;
    yInfo() << "Loading log file ... ";
    if(!loader.load(file_path)) {
        yError() << "Could not open log file";
        return false;
    } else {
        yInfo() << loader.getinfo();
    }

    std::string defaultpath = std::string(std::getenv("HOME")) + "/Downloads/events.mp4";
    std::string out_path = rf.check("out", Value(defaultpath)).asString();
    cv::VideoWriter dw;
    dw.open(out_path,
            cv::VideoWriter::fourcc('a','v','c','1'),
            fps*rate, res, true);

    double virtual_timer = period;
    if(stampfile.is_open()) {
        stampfile >> virtual_timer;
    } else {
        loader.synchroniseRealtimeRead(0.0);
    }

    if(rf.check("tw")) 
    {
        double duration = rf.check("window", Value(0.01)).asFloat64();

        while(loader.windowedReadTill(virtual_timer, duration) && !stampfile.eof()) {
            cv::Mat img = cv::Mat::zeros(res, CV_8UC3);

            for(auto &v : loader)
                img.at<cv::Vec3b>(v.y, v.x) = {255, 255, 255};
            if(vis) {
                cv::imshow("vLog2vid", img);
                cv::waitKey(1);
            }
            dw << img;
            if(stampfile.is_open()) stampfile >> virtual_timer;
            else virtual_timer += period;
            std::cout << "\r" << std::fixed << std::setprecision(1) << virtual_timer << " s / " << loader.getLength() << " s       ";
        }

    } else if(rf.check("scarf")) {

        ev::SCARF scarf;
        scarf.initialise(res, rf.check("block_size", Value(14)).asInt32(), rf.check("alpha", Value(1.0)).asFloat64(), rf.check("C", Value(0.3)).asFloat64());

        while(loader.incrementReadTill(virtual_timer) && !stampfile.eof()) {
            cv::Mat img, img8U;

            for(auto &v : loader)
                scarf.update(v.x, v.y, v.p);

            scarf.getSurface().convertTo(img8U, CV_8U, 255);
            img8U = 255 - img8U;
            cv::cvtColor(img8U, img, cv::COLOR_GRAY2BGR);

            if(vis) {
                cv::imshow("vLog2vid", img);
                cv::waitKey(1);
            }
            dw << img;
            if(stampfile.is_open()) stampfile >> virtual_timer;
            else virtual_timer += period;
            std::cout << "\r" << std::fixed << std::setprecision(1) << virtual_timer << " s / " << loader.getLength() << " s       ";
            std::cout.flush();
        }
    } else if(rf.check("eros")) {
        ev::EROS eros;
        eros.init(res.width, res.height, rf.check("block_size", Value(7)).asInt32(), rf.check("alpha", Value(0.3)).asFloat64());

        while(loader.incrementReadTill(virtual_timer) && !stampfile.eof()) {
            cv::Mat img, img8U;

            for(auto &v : loader)
                eros.update(v.x, v.y);

            eros.getSurface().copyTo(img8U);
            img8U = 255 - img8U;
            cv::cvtColor(img8U, img, cv::COLOR_GRAY2BGR);

            if(vis) {
                cv::imshow("vLog2vid", img);
                cv::waitKey(1);
            }
            dw << img;
            if(stampfile.is_open()) stampfile >> virtual_timer;
            else virtual_timer += period;
            std::cout << "\r" << std::fixed << std::setprecision(1) << virtual_timer << " s / " << loader.getLength() << " s       ";
            std::cout.flush();
        }
    } else {

        //initialise iso_drawer
        double duration = rf.check("window", Value(0.5)).asFloat64();
        ev::isoImager iso_drawer;
        cv::Size base_size = iso_drawer.init(res.height, res.width, duration);
        cv::Mat img = cv::Mat::zeros(res, CV_8UC3);
        cv::Mat base = cv::Mat::zeros(base_size, CV_8UC3);

        while(loader.windowedReadTill(virtual_timer, duration) && !stampfile.eof()) {
            
            base.setTo(ev::white);
            int count = 0;
            for(auto &v : loader) count++;
            iso_drawer.time_draw<ev::offlineLoader<ev::AE>::iterator>(base, loader.begin(), loader.end(), count);

            cv::resize(base, img, res);
            if(vis) {
                cv::imshow("vLog2vid", img);
                cv::waitKey(1);
            }

            dw << img;
            if(stampfile.is_open()) stampfile >> virtual_timer;
            else virtual_timer += period;
            std::cout << "\r" << std::fixed << std::setprecision(1) << virtual_timer << " s / " << loader.getLength() << " s       ";
            std::cout.flush();
        }
        
    }
    std::cout << std::endl;

    dw.release();

    return 0;

}