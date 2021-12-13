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

// \defgroup Visualisation Visualisation
// \defgroup vFramer vFramer
// \ingroup Visualisation
// \brief converts the event-stream to an yarpview-able image

#pragma once

#include <opencv2/opencv.hpp>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/cv/Cv.h>
#include <event-driven/core.h>
#include <event-driven/vis.h>
#include <map>
#include <vector>
#include <string>


class drawerInterface : public yarp::os::PeriodicThread {

protected:
    std::string name;
    cv::Mat canvas;
    yarp::os::BufferedPort< yarp::sig::FlexImage > image_port;

    void run() override;
    bool threadInit() override;
    virtual void updateImage() = 0;

public:

    drawerInterface();
    std::string drawerName();
    virtual bool initialise(const std::string &name, int height, int width)  = 0;

};

class greyDrawer : public drawerInterface {
protected:
    ev::window<ev::AE> input;
    ev::BufferedPort<ev::AE> temp_input;
    void updateImage() override;
public:
    bool initialise(const std::string &name, int height, int width) override;
};

class isoDrawer : public drawerInterface {
protected:

    static constexpr double time_window{1.0};
    ev::window<ev::AE> input;
    ev::pixelShifter ps;
    cv::Mat base_image;
    void updateImage() override;
    
public:
    bool initialise(const std::string &name, int height, int width) override;
};


// class overlayStereoDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class saeDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class grayDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class blackDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class binaryDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();
//     virtual void resetImage(cv::Mat &image);
// };

// class circleDraw : public vDraw {

// protected:

//     std::map<int, ev::event<ev::GaussianAE>> persistance;
//     int stagnantCount;

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class blobDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class accDraw : public vDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class isoCircDraw : public isoDraw {

// public:

//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };

// class rasterDrawHN : public vDraw {

// protected:

//     float time_scaler;
//     int neuron_min;
//     int neuron_max;
//     std::map<int, int> rmap;
//     std::vector<int> lines;
//     std::vector<string> texts;

// public:

//     virtual void initialise();
//     static const std::string drawtype;
//     virtual void draw(cv::Mat &image, const ev::vQueue &eSet, int vTime);
//     virtual std::string getDrawType();
//     virtual std::string getEventType();

// };
