/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren.Glover@iit.it
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

#include "vTrackToRobot.h"
#include "yarp/math/Math.h"
#include <algorithm>
#include <cmath>

using namespace yarp::math;
using namespace ev;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError() << "Could not connect to yarp";
        return -1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vGazeDemo.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vTrackToRobotModule module;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return module.runModule(rf);
}


/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

//    if(gazedriver.isValid() && dogaze && demo == graspdemo && gazingActive) {
//    //if(gazedriver.isValid() && demo == graspdemo && gazingActive) {

//        //this is the eye pose
//        yarp::sig::Vector xeye,oeye;
//        gazecontrol->getLeftEyePose(xeye,oeye);

//        //this does the transformation
//        yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
//        T(0,3)=xeye[0];
//        T(1,3)=xeye[1];
//        T(2,3)=xeye[2];
//        //std::cout << "initial rotation matrix" << std::endl;
//        //std::cout << T.toString() << std::endl;

//        //std::cout << "initial translations" << std::endl;
//        //std::cout << xeye.toString() << std::endl;

//        yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
//        //std::cout << "inverted rotation matrix" << std::endl;
//        //std::cout << Ti.toString() << std::endl;


//        //this was the target in eye coordinates
//        yarp::sig::Vector fp(4);
//        fp[0]=xrobref[0];
//        fp[1]=xrobref[1];
//        fp[2]=xrobref[2];
//        fp[3]=1.0;

//        //std::cout << "Multiplied by" << std::endl;
//        //std::cout << fp.toString() << std::endl;


//        yarp::sig::Vector tp=Ti*fp;
//        //std::cout << "Equals:" << std::endl;
//        //std::cout << tp.toString() << std::endl;
//        if(cartOutPort.getOutputCount()) {
//            yarp::os::Bottle &cartcoords = cartOutPort.prepare();
//            cartcoords.clear();
//            //    //add the XYZ position
//            cartcoords.add(tp[0]); cartcoords.add(tp[1]); cartcoords.add(tp[2]);
//            //cartcoords.add(-1.0); cartcoords.add(0.0); cartcoords.add(-0.3);
//            //    //add some buffer ints
//            cartcoords.add(0.5); cartcoords.add(px[0]); cartcoords.add(px[1]);
//            //    //flag that the object is detected
//            cartcoords.add(1.0);

//            //std::cout << "Bottle: " << cartcoords.toString() << std::endl;
//            //targetPos in the eye reference frame
//            //std::cout << "2D point: " << px.toString() << std::endl;
//            //std::cout << "3D point: " << x.toString() << std::endl;
//            cartOutPort.write();
//        }

//    }

bool vTrackToRobotModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    setName((rf.check("name", yarp::os::Value("/vGazeDemo")).asString()).c_str());
    yThresh = rf.check("yThresh", yarp::os::Value(20)).asDouble();
    rThresh = rf.check("rThresh", yarp::os::Value(5)).asDouble();
    period = rf.check("period", yarp::os::Value(0.01)).asDouble();
    gazingActive = rf.check("start", yarp::os::Value(false)).asBool();
    useDemoRedBall = rf.check("grasp", yarp::os::Value(false)).asBool();
    res.height = rf.check("height", yarp::os::Value(240)).asDouble();
    res.width = rf.check("width", yarp::os::Value(304)).asDouble();
    bool usearm = rf.check("arm") && rf.check("arm", yarp::os::Value(true)).asBool();

    if(!rpcPort.open(getName() + "/control"))
        return false;
    this->attach(rpcPort);

    //inputPort.setDemo(rf.check("demo", yarp::os::Value("gaze")).asString());

    if(!inputPort.open(getName() + "/vBottle:i"))
        return false;

    if(!cartOutPort.open(getName() + "/cart:o"))
        return false;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", getName());
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid()) {
        gazedriver.view(gazecontrol);
        gazecontrol->getHeadPose(headhomepos, headhomerot);
    } else {
        yWarning() << "Gaze Driver not opened and will not be used";
    }

    options.put("device","cartesiancontrollerclient");
    // left arm
    options.put("remote","/icub/cartesianController/left_arm");
    options.put("local","/cartesian_client/left_arm");
    //right arm
//    options.put("remote","/icub/cartesianController/right_arm");
//    options.put("local","/cartesian_client/right_arm");
    if(usearm) armdriver.open(options);
    if(armdriver.isValid()) {
        armdriver.view(arm);
        arm->storeContext(&startup_context_id);
        arm->setTrajTime(1.5);
        // get the torso dofs
        yarp::sig::Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        arm->setDOF(newDof,curDof);

        double min, max;
        // we keep the lower limit
        arm->getLimits(0,&min,&max);
        arm->setLimits(0,min,30.0);

        arm->getPose(armhomepos, armhomerot);

    } else
        yWarning() << "Arm driver not opened and will not be used";



    return true ;
}

bool vTrackToRobotModule::updateModule()
{

    static double htimeout = yarp::os::Time::now();
    if(yarp::os::Time::now() - htimeout > 3.0) {
        if(armdriver.isValid()) {
            arm->goToPose(armhomepos, armhomerot);
        }
        htimeout = yarp::os::Time::now();
    }


    yarp::sig::Vector leftTarget, rightTarget;
    inputPort.getTargets(leftTarget, rightTarget);

    if(!leftTarget[3] && !rightTarget[3]) {
        std::cout << "Weak signal from both cameras" << std::endl;
        return true;
    }
    if(!leftTarget[3]) {
        std::cout << "Weak signal from left camera" << std::endl;
        return true;
    }
    if(!rightTarget[3]) {
        std::cout << "Weak signal from right camera" << std::endl;
        return true;
    }

    //do our stereo target check
    if(std::abs(rightTarget[1] - leftTarget[1]) > yThresh) {
        //yWarning() << "Y values not consistent for target";
        return true;
    }
    if(std::abs(rightTarget[2] - leftTarget[2]) > rThresh) {
        //yWarning() << "Radius not consistent for target";
        return true;
    }

    if(!gazingActive || !gazedriver.isValid()) {
        //yInfo() << "Gaze valid (gazing blocked)";
        return true;
    }

    htimeout = yarp::os::Time::now();

    yarp::sig::Vector pleft = leftTarget.subVector(0, 1);
    //pleft[0] = res.width - 1 - pleft[0];
    //pleft[1] = res.height - 1 - pleft[1];

    yarp::sig::Vector pright = rightTarget.subVector(0, 1);
    //pright[0] = res.width - 1 - pright[0];
    //pright[1] = res.height - 1 - pright[1];

    if(!useDemoRedBall) {

        //yInfo() << "Doing gaze";
        yarp::sig::Vector tp;
        gazecontrol->triangulate3DPoint(pleft, pright, tp);


        //std::cout << tp.toString() << " " << std::abs(rightTarget[1] - leftTarget[1]) << " " << std::abs(rightTarget[2] - leftTarget[2]) << std::endl;
        //tp[0] = -0.09;
        if(tp[0] < -0.10) {
            //gazecontrol->lookAtMonoPixel(0, pleft, 0.5);
            gazecontrol->lookAtStereoPixels(pleft, pright);

            if(armdriver.isValid()) {

                tp[1] += -0.10;
                tp[2] += -0.10;

                //tp[0] = std::max(tp[0], -0.15);
                tp[0] = std::min(tp[0], -0.20);
                tp[0] = std::max(tp[0], -0.30);
                //tp[1] = std::max(tp[1], -0.6);
                tp[1] = std::min(tp[1],  0.10);
                //tp[2] = std::max(tp[2],  0.0); tp[2] = std::min(tp[2],  0.6);

                //arm->goToPosition(tp);


                yarp::sig::Vector od(4);
                yarp::sig::Matrix handor(3, 3); handor.zero();
                handor(0, 0) = -1.0; //hand x axis = - robot x axis (point forward)
                handor(2, 1) = -1.0;
                handor(1, 2) = 1.0;


                od = dcm2axis(handor);


                //od[0]=0.0; od[1]=0.5; od[2]=1.0; od[3]=M_PI;
                arm->goToPose(tp,od);

            }
        }

    } else {

        yarp::sig::Vector xrobref, xeye, oeye;
        gazecontrol->triangulate3DPoint(pleft, pright, xrobref);

        if(xrobref[0] < -0.15) {

            gazecontrol->getLeftEyePose(xeye,oeye); //in robot ref frame

            //from eye -> torso
            yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
            T(0,3)=xeye[0];
            T(1,3)=xeye[1];
            T(2,3)=xeye[2];

            //from torso -> eye
            yarp::sig::Matrix Ti = yarp::math::SE3inv(T);

            //this was the target in robot reference frame
            yarp::sig::Vector fp(4);
            fp[0]=xrobref[0];
            fp[1]=xrobref[1];
            fp[2]=xrobref[2];
            fp[3]=1.0;

            //convert point in robrf -> eyerf
            yarp::sig::Vector tp=Ti*fp;

            //send eye ref frame coordinates
            if(cartOutPort.getOutputCount()) {
                yarp::os::Bottle &cartcoords = cartOutPort.prepare();
                cartcoords.clear();
                //    //add the XYZ position
                cartcoords.add(tp[0]); cartcoords.add(tp[1]); cartcoords.add(tp[2]);
                //cartcoords.add(-1.0); cartcoords.add(0.0); cartcoords.add(-0.3);
                //    //add some buffer ints
                cartcoords.add(0.5); cartcoords.add(pleft[0]); cartcoords.add(pleft[1]);
                //    //flag that the object is detected
                cartcoords.add(1.0);

                cartOutPort.write();
            }

        }
    }

    return !isStopping();
}

double vTrackToRobotModule::getPeriod()
{
    return period;
}

bool vTrackToRobotModule::respond(const yarp::os::Bottle &command,
                                  yarp::os::Bottle &reply)
{
    reply.clear();

    if(command.get(0).asString() == "start") {
        reply.addString("starting");
        gazingActive = true;
    } else if(command.get(0).asString() == "stop") {
        reply.addString("stopping");
        gazingActive = false;
    } else {
        return false;
    }

    return true;


}

bool vTrackToRobotModule::interruptModule()
{

    if(armdriver.isValid()) {
        arm->goToPoseSync(armhomepos, armhomerot);
        arm->waitMotionDone(1.0, 4.0);
    }

    inputPort.interrupt();
    return yarp::os::RFModule::interruptModule();
}

bool vTrackToRobotModule::close()
{

    if(gazedriver.isValid()) {
        gazecontrol->stopControl();
        gazedriver.close();
    }

    if(armdriver.isValid()) {
        arm->stopControl();
        arm->restoreContext(startup_context_id);
        armdriver.close();
    }


    inputPort.close();
    return yarp::os::RFModule::close();
}

