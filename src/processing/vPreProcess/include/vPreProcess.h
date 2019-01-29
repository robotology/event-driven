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

// \defgroup Modules Modules
// \defgroup vPepper vPepper
// \ingroup Modules
// \brief removes salt-and-pepper noise from the event stream

#ifndef __VPREPROCESS__
#define __VPREPROCESS__

#define DECODE_METHOD 2

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
using namespace::ev;

class vPreProcess : public yarp::os::Thread
{
private:

    //output port for the vBottle with the new events computed by the module

    vReadPort < vector<int32_t> > inPort;
    vWritePort outPortCamLeft;
    vWritePort outPortCamRight;
    vWritePort outPortSkin;
    vWritePort outPortSkinSamples;

    //parameters
    std::string name;
    ev::resolution res;

    //pre-pre processing
    bool precheck;
    bool flipx;
    bool flipy;

    //filter class
    bool pepper;
    ev::vNoiseFilter thefilter;
    int v_total;
    int v_dropped;

    //we store an openCV map to use as a look-up table for the undistortion
    //given the camera parameters provided
    bool undistort;
    cv::Mat leftMap;
    cv::Mat rightMap;
    bool truncate;

    //output
    bool split;

    //timing stats
    std::deque<double> delays;
    std::deque<double> rates;
    std::deque<double> intervals;

public:

    vPreProcess();
    ~vPreProcess();

    void initBasic(std::string name, int height, int width, bool precheck,
                   bool flipx, bool flipy, bool pepper, bool undistort,
                   bool split);
    void initPepper(int spatialSize, int temporalSize);
    void initUndistortion(const yarp::os::Bottle &left,
                          const yarp::os::Bottle &right, bool truncate);
    int queryUnprocessed();
    std::deque<double> getDelays();
    std::deque<double> getRates();
    std::deque<double> getIntervals();
    void printFilterStats();
    void run();
    void onStop();
    bool threadInit();

};

class vPreProcessModule : public yarp::os::RFModule
{
    //the event bottle input and output handler
    vPreProcess      eventManager;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();

    virtual double getPeriod();
    virtual bool updateModule();

};


#endif

