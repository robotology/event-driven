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
#include <event-driven/algs.h>

#include <map>
#include <vector>
#include <string>


class drawerInterface : public yarp::os::PeriodicThread {

protected:
    std::string name;
    std::string portName;
    std::string sourceName;
    cv::Mat canvas;
    yarp::os::Stamp canvas_stamp;
    cv::Size img_size;
    bool yarp_publish;
    yarp::os::BufferedPort< yarp::sig::FlexImage > image_port;
    double window_size;

    void run() override;
    bool threadInit() override;
    virtual double updateImage() = 0;

public:

    drawerInterface() : PeriodicThread(0.05){};
    std::string drawerName();
    virtual bool initialise(const std::string &name, int height, int width, double window_size, 
                        bool yarp_publish = false, const std::string &remote = "") = 0;
    virtual void connectToRemote() {};

};

class drawerInterfaceAE : public drawerInterface 
{
protected:
    ev::window<ev::AE> input;
public:
    virtual bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
    void connectToRemote() override;
};

class greyDrawer : public drawerInterfaceAE {
protected:
    double updateImage() override;
public:
    greyDrawer(){window_size=0.1;}
};

class blackDrawer : public drawerInterfaceAE {
protected:
    double updateImage() override;
public:
    blackDrawer(){window_size=0.033;}
};

class flowDrawer : public drawerInterfaceAE {
protected:
    cv::Mat sae_p, sae_p_live;
    cv::Mat sae_n, sae_n_live;
    cv::Mat mask, mask_live;
    ev::vNoiseFilter nf;
    std::thread vt;
    void updateSAE();
    double tic, tic_live;

    ev::zrtFlow zrt_flow;
    ev::zcflow flow_rep;
    double updateImage() override;
public:
    flowDrawer(){window_size=0.033;};
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
};

class rtFlowDrawer : public drawerInterfaceAE {
protected:
    double rate{0.0};
    std::thread vt;
    cv::Mat sample;
    void updateSAE();

    ev::zrtFlow zrt_flow;
    double updateImage() override;
public:
    rtFlowDrawer(){window_size=0.033;};
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
};

class isoDrawer : public drawerInterfaceAE {
protected:
    ev::isoImager iso_drawer;
    double updateImage() override;
    
public:
    isoDrawer(){window_size=1.0;}
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
};

class erosDrawer : public drawerInterfaceAE {
protected:
    int kernelSize {5};
    double decay {0.3};
    ev::EROS EROS_vis;
    double updateImage() override;
    
public:
    erosDrawer(int kernelSize, double decay): kernelSize(kernelSize), decay(decay), drawerInterfaceAE(){};
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
};

class scarfDrawer : public drawerInterfaceAE {
protected:
    ev::SCARF scarf;
    std::thread vt;
    int meas_c{0};
    double meas_t{0.0};
    double scarf_time{0.0};
    int block{10};
    double alpha{1.0};
    double C{0.3};
    void updateScarfRep();
    double updateImage() override;
    
public:
    scarfDrawer(int block, double alpha, double C): block(block), alpha(alpha), C(C), drawerInterfaceAE(){};
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
};

class cornerDrawer : public drawerInterfaceAE {
protected:

    std::deque<ev::AE> corner_q;
    ev::isoImager iso_drawer;
    ev::corner_detector cd;
    double updateImage() override;
    void threadRelease() override;
    
public:
    
    cornerDrawer(){window_size=1.0;}
    bool initialise(const std::string &name, int height, int width, double window_size, bool yarp_publish, const std::string &remote = "") override;
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