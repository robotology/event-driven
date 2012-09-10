/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __MODULE_H__
#define __MODULE_H__

#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <cv.h>

#include "iCub/utils.h"

/**********************************************************/
class Manager : public yarp::os::RFModule
{
protected:

    std::string                 name;               //name of the module
    yarp::os::Port              rpcHuman;           //human rpc port (receive commands via rpc)
    yarp::os::RpcClient         rpcMotorAre;        //rpc motor port ARE
    yarp::os::RpcClient         rpcMotorKarma;      //rpc motor port KARMA    
    yarp::os::RpcClient         iolStateMachine;    //rpc to iol state machine
    yarp::os::RpcClient         rpcMIL;             //rpc mil port
    yarp::os::RpcClient         rpcTransTrad;       // rpc transformation traditional camera
    yarp::os::RpcClient         rpcTransEvent;      // rpc transformation event camera

    PointedLocation             pointedLoc;         //port class to receive pointed locations
    
    yarp::os::BufferedPort<yarp::os::Bottle>        blobExtractor;

    yarp::os::Semaphore         mutexResources;     //mutex for ressources
    CvPoint                     pointLocation;      //x and y of the pointed location
    bool                        init;
    bool                        pointGood;          //boolean for if got a point location
    yarp::os::Bottle            lastBlobs;
    
    std::string                 obj;
    double                      userTheta;

    std::map<int, double>       randActions;

    yarp::os::Bottle            getBlobs();
    CvPoint                     getBlobCOG(const yarp::os::Bottle &blobs, const int i);
    bool                        get3DPosition(const CvPoint &point, yarp::sig::Vector &x);
    yarp::os::Bottle            findClosestBlob(const yarp::os::Bottle &blobs, const CvPoint &loc);
    int                         processHumanCmd(const yarp::os::Bottle &cmd, yarp::os::Bottle &b);
    int                         executeOnLoc(bool shouldTrain);

    yarp::os::Bottle            getOffset(yarp::os::Bottle &closestBlob, double actionOrient, yarp::sig::Vector &initPos);
    
    void                        goHome();
    /**
     *      @brief: push an object detected with traditional cameras
     */
    void                        pushTraditional(int u, int v, double& x, double& y, double& z);     
    double                      wrapAng (const double ang);
    
    void                        acquireImage(const bool request=false);
    yarp::os::Bottle            classify(const yarp::os::Bottle &blobs, int index);
    yarp::os::Bottle            getType(const yarp::os::Bottle *mils, int index);

public:
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();
};
#endif

