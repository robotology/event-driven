/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
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

#include "vCornerModule.h"

using namespace ev;

/**********************************************************/
bool vCornerModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCorner")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(240)).asInt();
    int width = rf.check("width", yarp::os::Value(304)).asInt();
    int sobelsize = rf.check("filterSize", yarp::os::Value(5)).asInt();
    unsigned int qlen = rf.check("qsize", yarp::os::Value(36)).asInt();
    double temporalsize = rf.check("tempsize", yarp::os::Value(0.1)).asDouble();
    int windowRad = rf.check("spatial", yarp::os::Value(5)).asInt();
    double sigma = rf.check("sigma", yarp::os::Value(1.0)).asDouble();
    double thresh = rf.check("thresh", yarp::os::Value(8.0)).asDouble();
    bool callback = rf.check("callback", yarp::os::Value(false)).asBool();
    int nthreads = rf.check("nthreads", yarp::os::Value(2)).asInt();
    bool harris = rf.check("harris", yarp::os::Value(false)).asBool();
    bool fast = rf.check("fast", yarp::os::Value(true)).asBool();
    double gain = rf.check("gain", yarp::os::Value(0.1)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    if(callback) {

        harristhread = 0;
        fastthread = 0;

        if(harris) {
            fastcallback = 0;
            harriscallback = new vHarrisCallback(height, width, temporalsize, qlen, sobelsize, windowRad, sigma, thresh);
            return harriscallback->open(moduleName, strict);
        }
        else if(fast) {
            harriscallback = 0;
            fastcallback = new vFastCallback(height, width);
            return fastcallback->open(moduleName, strict);
        }
    }
    else {

        harriscallback = 0;
        fastcallback = 0;

        if(harris) {
            fastthread = 0;
            harristhread = new vHarrisThread(height, width, moduleName, strict, qlen, temporalsize,
                                             windowRad, sobelsize, sigma, thresh, nthreads, gain);
            if(!harristhread->start())
                return false;
        }
        else if(fast) {
            harristhread = 0;
            fastthread = new vFastThread(height, width, moduleName, strict, gain);
            if(!fastthread->start())
                return false;
        }
    }

    return true;
}

/**********************************************************/
bool vCornerModule::interruptModule()
{
    if(harriscallback) harriscallback->interrupt();
    if(fastcallback) fastcallback->interrupt();
    if(harristhread) harristhread->stop();
    if(fastthread) fastthread->stop();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCornerModule::close()
{
    if(harriscallback) {
        harriscallback->close();
        delete harriscallback;
    }
    if(fastcallback) {
        fastcallback->close();
        delete fastcallback;
    }
    if(harristhread) delete harristhread;
    if(fastthread) delete fastthread;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCornerModule::updateModule()
{
    return true;
}

/**********************************************************/
double vCornerModule::getPeriod()
{
    return 1;
}

//empty line to make gcc happy
