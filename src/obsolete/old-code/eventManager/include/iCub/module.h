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

    std::string                 name;               // name of the module
    yarp::os::Port              rpcHuman;           // human rpc port (receive commands via rpc)
    yarp::os::RpcClient         rpcMotorAre;        // rpc motor port ARE
    yarp::os::RpcClient         rpcMotorKarma;      // rpc motor port KARMA    
    yarp::os::RpcClient         iolStateMachine;    // rpc to iol state machine
    yarp::os::RpcClient         rpcMIL;             // rpc mil port
    yarp::os::RpcClient         rpcWBD;             // rpc wholeBody port
    yarp::os::RpcClient         rpcTransTrad;       // rpc transformation traditional camera
    yarp::os::RpcClient         rpcTransEvent;      // rpc transformation event camera
    yarp::os::RpcClient         rpcVelExtract;      // rpc for velocity extractor
    yarp::os::RpcClient         rpcAttention;       // rpc to the attentive system
    yarp::os::RpcClient         rpcGrabber;         // rpc to the grabber of events

    PointedLocation             pointedLoc;         //port class to receive pointed locations
    
    yarp::os::BufferedPort<yarp::os::Bottle>        blobExtractor;   //port where the visible blobs are seen
    yarp::os::BufferedPort<yarp::os::Bottle>        attentionFocus;  //port that receives attention redeployment commands

    yarp::os::Semaphore         mutexResources;     //mutex for ressources
    CvPoint                     pointLocation;      //x and y of the pointed location
    bool                        init;
    bool                        pointGood;          //boolean for if got a point location
    yarp::os::Bottle            lastBlobs;

    yarp::sig::Vector            leftDVS_kine;       // kinematic offset of the left camera DVS
    yarp::sig::Vector            rightDVS_kine;      // kinematic offset of the right cameara DVS
    yarp::sig::Vector            leftWINGS_kine;     // kinematic offset of the left camera on WINGS
    yarp::sig::Vector            rightWINGS_kine;    // kinematic offset of the right camera on WINGS
    
    std::string                 obj;
    double                      userTheta;

    std::map<int, double>       randActions;

    yarp::os::Bottle            getBlobs();
    CvPoint                     getBlobCOG(const yarp::os::Bottle &blobs, const int i);
    bool                        get3DPosition(const CvPoint &point, yarp::sig::Vector &x);
    yarp::os::Bottle            findClosestBlob(const yarp::os::Bottle &blobs, const CvPoint &loc);
    int                         processHumanCmd(const yarp::os::Bottle &cmd, yarp::os::Bottle &b);
    int                         executeOnLoc(bool shouldTrain);
    void                        wbdRecalibration();

    yarp::os::Bottle            getOffset(yarp::os::Bottle &closestBlob, double actionOrient, yarp::sig::Vector &initPos);
   
     /**
     *      @brief: basic command for touch in are
     */
    void                        touch(double x, double y, double z);     
    /**
     *      @brief : basic command for home in are
     */
    void                        goHome();
    /**
     *      @brief: push an object detected with traditional cameras
     */
    void                        pushTraditional(int u, int v, double& x, double& y, double& z);     
    /**
     *      @brief: push an object detected with traditional cameras enabling smooth pursuit right after
     *      @param u position on the image plane of traditional cameras
     *      @param v position on the image plane of traditional cameras
     */
    void                        pushSmoothPursuit(int u, int v);
     /**
     *      @brief: point to an object detected with traditional cameras enabling smooth pursuit right after
     *      @param u position on the image plane of traditional cameras
     *      @param v position on the image plane of traditional cameras
     */
    int                        pointDVS(int u, int v);
    /**
     *      @brief: touch an object detected with DVS cameras 
     *      @param u position on the image plane of traditional cameras
     *      @param v position on the image plane of traditional cameras
     */
    void                        touchDVS(int u, int v);

    /**
     *      @brief: touch an object detected with DVS cameras 
     *      @param u position on the image plane of traditional cameras
     *      @param v position on the image plane of traditional cameras
     */
    int                        learnDVS(int u, int v);
    /**
     *      @brief  : push an object detected with traditional cameras enabling smooth pursuit right after
     *      @return : result of the action
     */
    int                        pushOnLoc();
    /**
     *      @brief  : push an object detected with traditional cameras enabling smooth pursuit right after     
     *      @return : result of the action
     */
    int                        pursOnLoc();
    /**
     *      @brief  : touch an object detected with traditional cameras enabling smooth pursuit right after     
     *      @return : result of the action
     */
    int                        touchOnLoc();
    
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

