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

#include "vPreProcess.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not find YARP";
        return false;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vPreProcess.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vPreProcessModule preProcessModule;
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return preProcessModule.runModule(rf);
}

/******************************************************************************/
bool vPreProcessModule::configure(yarp::os::ResourceFinder &rf)
{
    bool pepper = rf.check("pepper") &&
            rf.check("pepper", yarp::os::Value(true)).asBool();
    bool undistort = rf.check("undistort") &&
            rf.check("undistort", yarp::os::Value(true)).asBool();
    bool truncate = rf.check("truncate") &&
            rf.check("truncate", yarp::os::Value(true)).asBool();
    bool flipx = rf.check("flipx") &&
            rf.check("flipx", yarp::os::Value(true)).asBool();
    bool flipy = rf.check("flipy") &&
            rf.check("flipy", yarp::os::Value(true)).asBool();
    bool precheck = rf.check("precheck") &&
            rf.check("precheck", yarp::os::Value(true)).asBool();
    bool split = rf.check("split") &&
            rf.check("precheck", yarp::os::Value(true)).asBool();

    if(precheck)
        yInfo() << "Performing precheck for event corruption";
    if(flipx)
        yInfo() << "Flipping vision horizontally";
    if(flipy)
        yInfo() << "Flipping vision vertically";
    if(pepper)
        yInfo() << "Applying salt and pepper filter";
    if(undistort && truncate)
        yInfo() << "Applying camera undistortion - truncating to sensor size";
    if(undistort && !truncate)
        yInfo() << "Applying camera undistortion - without truncation";
    if(split)
        yInfo() << "Splitting into left/right streams";


    eventManager.initBasic(rf.check("name", yarp::os::Value("/vPreProcess")).asString(),
                           rf.check("height", 240).asInt(),
                           rf.check("width", 304).asInt(),
                           precheck, flipx, flipy, pepper, undistort, split);

    if(pepper) {
        eventManager.initPepper(rf.check("spatialSize", yarp::os::Value(1)).asDouble(),
                                rf.check("temporalSize", yarp::os::Value(100000)).asDouble());
    }

    if(undistort) {
        yarp::os::ResourceFinder calibfinder;
        calibfinder.setVerbose();
        calibfinder.setDefaultContext(rf.check("calibContext", yarp::os::Value("cameraCalibration")).asString().c_str());
        calibfinder.setDefaultConfigFile(rf.check("calibFile", yarp::os::Value("cameraCalibration")).asString().c_str());
        calibfinder.configure(0, 0);

        yarp::os::Bottle &leftParams = calibfinder.findGroup("CAMERA_CALIBRATION_LEFT");
        yarp::os::Bottle &rightParams = calibfinder.findGroup("CAMERA_CALIBRATION_RIGHT");
        if(leftParams.isNull() || rightParams.isNull()) {
            yError() << "Could not load camera parameters";
            return false;
        }
        std::cout << leftParams.toString() << std::endl;
        std::cout << rightParams.toString() << std::endl;
        eventManager.initUndistortion(leftParams, rightParams, truncate);
    }

    return eventManager.start();

}

bool vPreProcessModule::close()
{
    eventManager.stop();
    return yarp::os::RFModule::close();
}

bool vPreProcessModule::updateModule()
{
    return true;
}

double vPreProcessModule::getPeriod()
{
    yInfo() << this->eventManager.queryUnprocessed() << "unprocessed queues";
    return 3.0;
}
/******************************************************************************/
vPreProcess::vPreProcess(): name("/vPreProcess")
{
    leftMap.deallocate();
    rightMap.deallocate();
}


vPreProcess::~vPreProcess()
{
    outPort.close();
    outPort2.close();
}

void vPreProcess::initBasic(std::string name, int height, int width,
                            bool precheck, bool flipx, bool flipy,
                            bool pepper, bool undistort, bool split)
{

    this->name = name;
    res.height = height;
    res.width = width;
    this->precheck = precheck;
    this->flipx = flipx;
    this->flipy = flipy;
    this->pepper = pepper;
    this->undistort = undistort;
    this->split = split;

}

void vPreProcess::initPepper(int spatialSize, int temporalSize)
{
    thefilter.initialise(res.width, res.height, temporalSize, spatialSize);
}

void vPreProcess::initUndistortion(const yarp::os::Bottle &left,
                               const yarp::os::Bottle &right, bool truncate)
{
    this->truncate = truncate;
    const yarp::os::Bottle *coeffs[2] = { &left, &right};
    cv::Mat *maps[2] = {&leftMap, &rightMap};

    //create camera matrix
    for(int i = 0; i < 2; i++) {

        double scaley = res.height / (double)(coeffs[i]->find("h").asInt());
        double scalex = res.width  / (double)(coeffs[i]->find("w").asInt());

        cv::Mat cameraMatrix(3, 3, CV_64FC1);
        cameraMatrix.setTo(0);
        cameraMatrix.at<double>(0, 0) = coeffs[i]->find("fx").asDouble()*scalex;
        cameraMatrix.at<double>(1, 1) = coeffs[i]->find("fy").asDouble()*scaley;
        cameraMatrix.at<double>(2, 2) = 1.0;
        cameraMatrix.at<double>(0, 2) = coeffs[i]->find("cx").asDouble()*scalex;
        cameraMatrix.at<double>(1, 2) = coeffs[i]->find("cy").asDouble()*scaley;

        cv::Mat distCoeffs(4, 1, CV_64FC1);
        distCoeffs.at<double>(0, 0) = coeffs[i]->find("k1").asDouble();
        distCoeffs.at<double>(0, 1) = coeffs[i]->find("k2").asDouble();
        distCoeffs.at<double>(0, 2) = coeffs[i]->find("p1").asDouble();
        distCoeffs.at<double>(0, 3) = coeffs[i]->find("p2").asDouble();


        cv::Mat allpoints(res.height * res.width, 1, CV_32FC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                allpoints.at<cv::Vec2f>(y * res.width + x) = cv::Vec2f(x, y);
            }
        }

        cv::Mat mappoints(res.height * res.width, 1, CV_32FC2);
        cv::Size s(res.height, res.width);
        cv::Mat defCamMat = cv::getDefaultNewCameraMatrix(cameraMatrix, s,
                                                          true);
        cv::undistortPoints(allpoints, mappoints, cameraMatrix, distCoeffs,
                            cv::noArray(), defCamMat);

        *(maps[i]) = cv::Mat(res.height, res.width, CV_32SC2);
        for(unsigned int y = 0; y < res.height; y++) {
            for(unsigned int x = 0; x < res.width; x++) {
                maps[i]->at<cv::Vec2i>(y, x) =
                        mappoints.at<cv::Vec2f>(y * res.width + x);
            }
        }
    }

}

int vPreProcess::queryUnprocessed()
{
    return inPort.queryunprocessed();
}

void vPreProcess::run()
{
    yarp::os::Stamp ystamp;

    resolution resmod = res;
    resmod.height -= 1;
    resmod.width -= 1;
    int prev_bottle_n = 0;

    //left output data
    vBottleMimic leftBottle;
    leftBottle.setHeader(AE::tag);

    //right output data
    vBottleMimic rightBottle;
    rightBottle.setHeader(AE::tag);

    while(true) {

        vQueue qleft, qright;

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inPort.getNextQ(ystamp);
        }
        if(isStopping()) break;

        if(precheck && prev_bottle_n + 1 != ystamp.getCount() && ystamp.getCount() && prev_bottle_n) {
            yWarning() << "Dropped bottle:" << prev_bottle_n << "to" << ystamp.getCount();
        }
        prev_bottle_n = ystamp.getCount();

        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            auto v = is_event<AE>(*qi);

            //precheck
            if(precheck && (v->x < 0 || v->x > resmod.width || v->y < 0 || v->y > resmod.height)) {
                yWarning() << "Event Corruption:" << v->getContent().toString();
                continue;
            }

            //flipx
            if(flipx) v->x = resmod.width - v->x;
            //flipy
            if(flipy) v->y = resmod.height - v->y;

            //salt and pepper filter
            if(pepper && !thefilter.check(v->x, v->y, v->polarity, v->channel, v->stamp))
                continue;

            //undistortion
            if(undistort) {
                cv::Vec2i mapPix;
                if(v->getChannel() == 0)
                    mapPix = leftMap.at<cv::Vec2i>(v->y, v->x);
                else
                    mapPix = rightMap.at<cv::Vec2i>(v->y, v->x);
                v->x = mapPix[0];
                v->y = mapPix[1];

                //truncate to sensor bounds after mapping?
                if(truncate && (v->x < 0 || v->x > resmod.width || v->y < 0 || v->y > resmod.height)) {
                    continue;
                }

            }

            if(split && v->channel)
                qright.push_back(v);
            else
                qleft.push_back(v);
        }

        if(qleft.size()) {
            leftBottle.setInternalData(qleft);
            outPort.setEnvelope(ystamp);
            outPort.write(leftBottle);
        }
        if(qright.size()) {
            rightBottle.setInternalData(qright);
            outPort2.setEnvelope(ystamp);
            outPort2.write(rightBottle);
        }

        inPort.scrapQ();
    }

}

void vPreProcess::onStop()
{
    outPort.close();
    outPort2.close();
    inPort.close();
    inPort.releaseDataLock();
}

bool vPreProcess::threadInit()
{
    if(split) {
        if(!outPort.open(name + "/left:o"))
            return false;
        if(!outPort2.open(name + "/right:o"))
            return false;
    } else {
        if(!outPort.open(name + "/vBottle:o"))
            return false;
    }
    if(!inPort.open(name + "/vBottle:i"))
        return false;
    return true;
}

