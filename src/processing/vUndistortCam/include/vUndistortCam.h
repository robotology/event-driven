/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren Glover
 * email:  arren.glover@iit.it
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

/// \defgroup processing Processing
/// \defgroup vUndistortCam vUndistortCam
/// \ingroup processing
/// \brief undistort the DVS camera using camera calibration parameters

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <opencv/cv.h>

class EventBottleManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    bool strictio;

    //we store an openCV map to use as a look-up table for the undistortion
    //given the camera parameters provided
    cv::Mat leftMap;
    cv::Mat rightMap;

    //sensor resolution: default 128x128
    unsigned int sensorHeight;
    unsigned int sensorWidth;

    //do we send on data remapped to outside the sensor bounds?
    bool truncate;

public:

    EventBottleManager();

    void setSensorSize(int sensorHeight, int sensorWidth) {
        this->sensorHeight = sensorHeight;
        this->sensorWidth = sensorWidth;
    }

    void setToTruncate(bool truncate) {
        this->truncate = truncate;
    }

    void setCamParams(const yarp::os::Bottle &left,
                      const yarp::os::Bottle &right);

    bool    open(const std::string &name, bool strictio);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);

};

class vUndistortModule : public yarp::os::RFModule
{
    //the remote procedure port
    yarp::os::RpcServer     rpcPort;

    //the event bottle input and output handler
    EventBottleManager      eventBottleManager;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
