/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <string.h>

#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <iCub/emorph/svm.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class vBottleManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    //name of the module (rootname of ports)
    std::string moduleName;

    //input port for the encoders
    yarp::os::BufferedPort<yarp::os::Bottle> inPortEncoders;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    //for helping with timestamp wrap around
    emorph::vtsHelper unwrapper;

    /******************************************************************************/
    //   FUNCTIONS
    /******************************************************************************/
    double compute_and_scale_vel(double last_pos, double curr_pos, double dt);
    std::vector<double> predict_mean(svm_node *test);
    cv::Mat predict_cov(svm_node *test);

    /******************************************************************************/
    //   VARIABLES
    /******************************************************************************/
    double threshold;
    bool first;

    double last_joint_pos[6] = {0, 0, 0, 0, 0, 0};

    std::ofstream saveFile;
    std::stringstream line2save;

    uint x;
    uint y;
    float vx;
    float vy;
    uint ts;

public:
    
    vBottleManager(const std::string &_moduleName, double &_threshold);

    bool    open();
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &vbot);

};

class vEgoMotionModule : public yarp::os::RFModule
{

    //the remote procedure port
    yarp::os::RpcServer     rpcPort;

    //the vbottle input and output handler
    vBottleManager      *vBotManager;


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
