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

#include "vUndistortCam.h"

/**********************************************************/
bool vUndistortModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name", yarp::os::Value("vTemplate"),
                                      "module name (string)").asString();
    setName(moduleName.c_str());

    //open and attach the rpc port
    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << getName() << " : Unable to open rpc port at " <<
                     rpcPortName << std::endl;
        return false;
    }

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcPort);

    //set sensor size
    int height = rf.check("sensorHeight",
                          yarp::os::Value(128),
                          "camera resolution height").asInt();
    int width = rf.check("sensorWidth",
                         yarp::os::Value(128),
                         "camera resolution width").asInt();

    eventBottleManager.setSensorSize(height, width);


    std::cout << rf.toString() << std::endl;
    //set other variables we need from the
    yarp::os::Bottle &leftParams = rf.findGroup("CAMERA_CALIBRATION_LEFT");

    yarp::os::Bottle &rightParams = rf.findGroup("CAMERA_CALIBRATION_RIGHT");

    std::cout << leftParams.toString() << std::endl;
    std::cout << rightParams.toString() << std::endl;
    //make sure we loaded something
    if(leftParams.isNull() || rightParams.isNull()) {
        std::cerr << "Could not load camera parameters" << std::endl;
        return false;
    }
    eventBottleManager.setCamParams(leftParams, rightParams);

    //open our bottle manager
    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool vUndistortModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    return true;
}

/**********************************************************/
bool vUndistortModule::close()
{
    rpcPort.close();
    eventBottleManager.close();
    return true;
}

/**********************************************************/
bool vUndistortModule::updateModule()
{
    return true;
}

/**********************************************************/
double vUndistortModule::getPeriod()
{
    return 0.1;
}

bool vUndistortModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    leftMap.deallocate();
    rightMap.deallocate();

    sensorHeight = 128;
    sensorWidth = 128;

    truncate = true;

}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);
    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    outPort.close();
    this->close();

    //remember to also deallocate any memory allocated by this class


}

/******************************************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    this->interrupt();

}
/******************************************************************************/
bool EventBottleManager::setCamParams(const yarp::os::Bottle &left,
                                      const yarp::os::Bottle &right)
{

    const yarp::os::Bottle *coeffs[2] = { &left, &right};
    cv::Mat *maps[2] = {&leftMap, &rightMap};

    //create camera matrix
    for(int i = 0; i < 2; i++) {

        double scaley = sensorHeight / (double)(coeffs[i]->find("h").asInt());
        double scalex = sensorWidth  / (double)(coeffs[i]->find("w").asInt());

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


        cv::Mat allpoints(sensorHeight * sensorWidth, 1, CV_32FC2);
        for(int y = 0; y < sensorHeight; y++) {
            for(int x = 0; x < sensorWidth; x++) {
                allpoints.at<cv::Vec2f>(y * sensorWidth + x) = cv::Vec2f(x, y);
            }
        }

        cv::Mat mappoints(sensorHeight * sensorWidth, 1, CV_32FC2);
        cv::Size s(sensorHeight, sensorWidth);
        cv::Mat defCamMat = cv::getDefaultNewCameraMatrix(cameraMatrix, s,
                                                          true);
        cv::undistortPoints(allpoints, mappoints, cameraMatrix, distCoeffs,
                            cv::noArray(), defCamMat);

        *(maps[i]) = cv::Mat(sensorHeight, sensorWidth, CV_32SC2);
        for(int y = 0; y < sensorHeight; y++) {
            for(int x = 0; x < sensorWidth; x++) {
                maps[i]->at<cv::Vec2i>(x, y) =
                        mappoints.at<cv::Vec2f>(y * sensorWidth + x);
//                std::cout << "[" << x << ", " << y << "] -> " <<
//                             maps[i]->at<cv::Vec2i>(x, y) << std::endl;
            }
        }
    }

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{

    //if we haven't loaded our camera parameters properly don't do anything
    if(leftMap.empty() || rightMap.empty()) {
        return;
    }

    //create event queue
    emorph::vQueue q = bot.getAll();
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    // get the event queue in the vBottle bot
    //bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent * v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;

        cv::Vec2i mapPix;
        if(!v->getChannel())
             mapPix = leftMap.at<cv::Vec2i>(v->getX(), v->getY());
        else
            mapPix = rightMap.at<cv::Vec2i>(v->getX(), v->getY());

        bool withinSensorBounds = mapPix[0] >= 0 && mapPix[0] < sensorWidth
                && mapPix[1] >= 0 && mapPix[1] < sensorHeight;
        if(withinSensorBounds || !truncate) {
            v->setX(mapPix[0]);
            v->setY(mapPix[1]);
            outBottle.addEvent(*v);
        }


    }
    //send on the processed events
    outPort.write();

}

//empty line to make gcc happy
