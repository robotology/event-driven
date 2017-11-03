/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

// \defgroup Modules Modules
// \defgroup vPepper vPepper
// \ingroup Modules
// \brief removes salt-and-pepper noise from the event stream

#ifndef __VPREPROCESS__
#define __VPREPROCESS__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <opencv/cv.h>
using namespace::ev;

class vPreProcess : public yarp::os::Thread
{
private:

    //output port for the vBottle with the new events computed by the module
    ev::queueAllocator inPort;
    yarp::os::Port outPort;
    yarp::os::Port outPort2;

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

    //we store an openCV map to use as a look-up table for the undistortion
    //given the camera parameters provided
    bool undistort;
    cv::Mat leftMap;
    cv::Mat rightMap;
    bool truncate;

    //output
    bool split;

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

