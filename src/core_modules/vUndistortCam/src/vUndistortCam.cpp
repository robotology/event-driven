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

using namespace ev;

/**********************************************************/
bool vUndistortModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("/vUndistortCam")).asString();
    setName(moduleName.c_str());

    //set port strictness
    bool strictio = rf.check("strict");

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
    eventBottleManager.open(moduleName, strictio);

    return true ;
}

/**********************************************************/
bool vUndistortModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vUndistortModule::close()
{
    rpcPort.close();
    eventBottleManager.close();
    yarp::os::RFModule::close();
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
    return 0.5;
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
    strictio = false;

}
/**********************************************************/
bool EventBottleManager::open(const std::string &name, bool strictio)
{
    //and open the input port

    if(strictio) this->setStrict();
    this->strictio = strictio;
    this->useCallback();

    yarp::os::BufferedPort<ev::vBottle>::open("/" + name + "/vBottle:i");
    outPort.open("/" + name + "/vBottle:o");
    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/******************************************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}
/******************************************************************************/
void EventBottleManager::setCamParams(const yarp::os::Bottle &left,
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
        for(unsigned int y = 0; y < sensorHeight; y++) {
            for(unsigned int x = 0; x < sensorWidth; x++) {
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
        for(unsigned int y = 0; y < sensorHeight; y++) {
            for(unsigned int x = 0; x < sensorWidth; x++) {
                maps[i]->at<cv::Vec2i>(x, y) =
                        mappoints.at<cv::Vec2f>(y * sensorWidth + x);
//                std::cout << "[" << x << ", " << y << "] -> " <<
//                             maps[i]->at<cv::Vec2i>(x, y) << std::endl;
            }
        }
    }

}

/**********************************************************/
void EventBottleManager::onRead(ev::vBottle &bot)
{

    //if we haven't loaded our camera parameters properly don't do anything
    if(leftMap.empty() || rightMap.empty()) {
        return;
    }

    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    //push the envelope through
    yarp::os::Stamp yst;
    this->getEnvelope(yst);
    outPort.setEnvelope(yst);

    //create event queue
    vQueue q = bot.get<AE>();

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        auto v = ev::is_event<AddressEvent>(*qi);

        cv::Vec2i mapPix;
        if(!v->getChannel())
             mapPix = leftMap.at<cv::Vec2i>(v->x, v->y);
        else
            mapPix = rightMap.at<cv::Vec2i>(v->x, v->y);

        bool withinSensorBounds = mapPix[0] >= 0.0 && mapPix[0] < (double)sensorWidth
                && mapPix[1] >= 0.0 && mapPix[1] < (double)sensorHeight;
        if(withinSensorBounds || !truncate) {
            v->x = mapPix[0];
            v->y = mapPix[1];
            outBottle.addEvent(v);
        }


    }
    //send on the processed events
    if(strictio) outPort.writeStrict();
    else outPort.write();

}

//empty line to make gcc happy
