/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: valentina.vasco@iit.it
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
    int height = rf.check("height", yarp::os::Value(240)).asInt32();
    int width = rf.check("width", yarp::os::Value(304)).asInt32();
    int sobelsize = rf.check("filterSize", yarp::os::Value(5)).asInt32();
    unsigned int qlen = rf.check("qsize", yarp::os::Value(36)).asInt32();
    double temporalsize = rf.check("tempsize", yarp::os::Value(0.1)).asFloat64();
    int windowRad = rf.check("spatial", yarp::os::Value(5)).asInt32();
    double sigma = rf.check("sigma", yarp::os::Value(1.0)).asFloat64();
    double thresh = rf.check("thresh", yarp::os::Value(8.0)).asFloat64();
    bool callback = rf.check("callback", yarp::os::Value(false)).asBool();
    int nthreads = rf.check("nthreads", yarp::os::Value(2)).asInt32();
    double gain = rf.check("gain", yarp::os::Value(0.1)).asFloat64();

    /* create the thread and pass pointers to the module parameters */
    if(callback) {
        harristhread = 0;
        harriscallback = new vHarrisCallback(height, width, temporalsize, qlen, sobelsize, windowRad, sigma, thresh);
        return harriscallback->open(moduleName, strict);
    }
    else {
        harriscallback = 0;
        harristhread = new vHarrisThread(height, width, moduleName, strict, qlen, temporalsize,
                                         windowRad, sobelsize, sigma, thresh, nthreads, gain);
        if(!harristhread->start())
            return false;
    }

    return true;
}

/**********************************************************/
bool vCornerModule::interruptModule()
{
    if(harriscallback) harriscallback->interrupt();
    if(harristhread) harristhread->stop();
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
    if(harristhread) delete harristhread;
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
