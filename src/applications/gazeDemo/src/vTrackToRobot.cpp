/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
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

bool vTrackToRobotModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    setName((rf.check("name", yarp::os::Value("/vGazeDemo")).asString()).c_str());
    yThresh = rf.check("yThresh", yarp::os::Value(20)).asDouble();
    rThresh = rf.check("rThresh", yarp::os::Value(5)).asDouble();
    period = rf.check("period", yarp::os::Value(0.01)).asDouble();
    gazingActive = rf.check("start", yarp::os::Value(false)).asBool();
    useDemoRedBall = rf.check("grasp", yarp::os::Value(false)).asBool();
    velocityControl = rf.check("velocity", yarp::os::Value(false)).asBool();
    res.height = rf.check("height", yarp::os::Value(240)).asDouble();
    res.width = rf.check("width", yarp::os::Value(304)).asDouble();
    usearm = rf.check("arm") && rf.check("arm", yarp::os::Value(true)).asBool();

    arm_target_position = yarp::sig::Vector(3);
    arm_target_position = 0.0;

    if(!rpcPort.open(getName() + "/control"))
        return false;
    this->attach(rpcPort);

    //inputPort.setDemo(rf.check("demo", yarp::os::Value("gaze")).asString());

    if(!inputPort.open(getName() + "/vBottle:i"))
        return false;

    if(!cartOutPort.open(getName() + "/cart:o"))
        return false;

    if(!debugOutPort.open(getName() + "/debug:o"))
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
        arm->setTrajTime(1.0);
        // get the torso dofs
        yarp::sig::Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=1;
        arm->setDOF(newDof,curDof);

        double min, max;
        // we keep the lower limit
        arm->getLimits(0,&min,&max);
        arm->setLimits(0,min,30.0);

        arm->getPose(armhomepos, armhomerot);

    } else
        yWarning() << "Arm driver not opened and will not be used";

    if(velocityControl) {
        yInfo() << "Velocity control in visual space";
        if(!velocityController.initialise(res.height, res.width))
            return false;
    } else {
        yInfo() << "Position-based cartesian control";
    }


    return true ;
}

bool vTrackToRobotModule::controlCartesian(yarp::sig::Vector ltarget,
                                           yarp::sig::Vector rtarget)
{
    //check target is present
    if(!rtarget[3] || !ltarget[3])
        return false;

    //do our stereo target check
    if(std::abs(rtarget[1] - ltarget[1]) > yThresh) {
        //yWarning() << "Y values not consistent for target";
        return false;
    }
    if(std::abs(rtarget[2] - ltarget[2]) > rThresh) {
        //yWarning() << "Radius not consistent for target";
        return false;
    }

    if(!gazingActive || !gazedriver.isValid()) {
        //yInfo() << "Gaze valid (gazing blocked)";
        return false;
    }

    yarp::sig::Vector pleft = ltarget.subVector(0, 1);
    yarp::sig::Vector pright = rtarget.subVector(0, 1);

    yarp::sig::Vector tp;
    gazecontrol->triangulate3DPoint(pleft, pright, tp);

    //target too close to body?
//    if(tp[0] > -0.10) {
//        return false;
//    }

    gazecontrol->lookAtStereoPixels(pleft, pright);

    return true;

}

bool vTrackToRobotModule::controlArm(yarp::sig::Vector ltarget,
                                     yarp::sig::Vector rtarget)
{
    //check target is present
    if(!rtarget[3] || !ltarget[3])
        return false;

    //do our stereo target check
    if(std::abs(rtarget[1] - ltarget[1]) > yThresh) {
        //yWarning() << "Y values not consistent for target";
        return false;
    }
    if(std::abs(rtarget[2] - ltarget[2]) > rThresh) {
        //yWarning() << "Radius not consistent for target";
        return false;
    }

    if(!gazingActive || !gazedriver.isValid()) {
        //yInfo() << "Gaze valid (gazing blocked)";
        return false;
    }

    if(!armdriver.isValid())
        return false;

    yarp::sig::Vector pleft = ltarget.subVector(0, 1);
    yarp::sig::Vector pright = rtarget.subVector(0, 1);

    yarp::sig::Vector tp;
    gazecontrol->triangulate3DPoint(pleft, pright, tp);



    double DCONST = 0.3;
    tp[0] = -DCONST;

    if(fabs(tp[1]) < 0.1) {
        tp[0] = -DCONST;
    } else {
        double sign = tp[1] < 0 ? -1 : 1;
        double temp = sqrt(((tp[0] * tp[0]) / (tp[1] * tp[1])) + 1);
        double yhat = sign * DCONST / temp;
        double xhat = tp[0] * (yhat / tp[1]);
        tp[0] = xhat;
        tp[1] = yhat;
    }

    //tp[0] += 0.1;
    tp[2] += -0.15;
    tp[2] = std::max(tp[2], 0.0);

    //std::cout << tp.toString() << std::endl;


    //target too close to body
    //if(tp[0] > -0.10) {
    //    return false;
    //}


    //tp[2] += -0.10;

    //tp[0] = std::max(tp[0], -0.15);
    //tp[0] = std::min(tp[0], -0.15);
    //tp[0] = std::max(tp[0], -0.30);
    //tp[1] = std::max(tp[1], -0.6);
    //tp[1] = std::min(tp[1],  0.10);
    //tp[2] = std::max(tp[2],  0.0); tp[2] = std::min(tp[2],  0.6);

    //tp[0] = -0.3;
    yarp::sig::Vector od(4);
    yarp::sig::Matrix handor(3, 3); handor.zero();
    handor(0, 0) = -1.0; //hand x axis = - robot x axis (point forward)
    handor(2, 1) = -1.0;
    handor(1, 2) = 1.0;

    od = dcm2axis(handor);

    //arm->goToPose(tp,od);
    arm->goToPosition(tp);
    arm_target_position = tp;
    //yInfo() << "Arm Controlled";

    return true;

}

bool vTrackToRobotModule::controlVelocity(yarp::sig::Vector ltarget,
                                          yarp::sig::Vector rtarget)
{
    static double trecord = yarp::os::Time::now();
    double dt = yarp::os::Time::now() - trecord;
    trecord += dt;

//    //check target is present
//    if(!rtarget[3] || !ltarget[3]) {
//        velocityController.controlReset();
//        return false;
//    }

//    bool sameY = std::abs(rtarget[1] - ltarget[1]) < yThresh;
//    bool sameR = std::abs(rtarget[2] - ltarget[2]) < rThresh;
//    if(!sameY || !sameR) {
//        velocityController.controlReset();
//        return false;
//    }

//    velocityController.controlStereo(ltarget[0], ltarget[1], rtarget[0],
//            rtarget[1], dt);



    if(ltarget[3]) {
        bool sameY = std::abs(rtarget[1] - ltarget[1]) < yThresh;
        bool sameR = std::abs(rtarget[2] - ltarget[2]) < rThresh;
        if(rtarget[3] && sameY && sameR)
            velocityController.controlStereo(ltarget[0], ltarget[1], rtarget[0],
                    rtarget[1], dt);
        else
            velocityController.controlMono(ltarget[0], ltarget[1], dt);
    } else if(rtarget[3])
        velocityController.controlMono(rtarget[0], rtarget[1], dt);
    else {
        velocityController.controlReset();
        return false;
    }

    return true;
}

bool vTrackToRobotModule::controlExternal(yarp::sig::Vector ltarget,
                                          yarp::sig::Vector rtarget)
{

    //check target is present
    if(!rtarget[3] || !ltarget[3])
        return false;

    //do our stereo target check
    if(std::abs(rtarget[1] - ltarget[1]) > yThresh) {
        //yWarning() << "Y values not consistent for target";
        return false;
    }
    if(std::abs(rtarget[2] - ltarget[2]) > rThresh) {
        //yWarning() << "Radius not consistent for target";
        return false;
    }

    if(!gazingActive || !gazedriver.isValid()) {
        //yInfo() << "Gaze valid (gazing blocked)";
        return false;
    }

    if(!armdriver.isValid())
        return false;

    yarp::sig::Vector pleft = ltarget.subVector(0, 1);
    yarp::sig::Vector pright = rtarget.subVector(0, 1);

    yarp::sig::Vector xrobref, xeye, oeye;
    gazecontrol->triangulate3DPoint(pleft, pright, xrobref);

    //target too close to body?
    if(xrobref[0] > -0.10) {
        return false;
    }

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
        cartcoords.addDouble(tp[0]); cartcoords.addDouble(tp[1]); cartcoords.addDouble(tp[2]);
        //cartcoords.add(-1.0); cartcoords.add(0.0); cartcoords.add(-0.3);
        //    //add some buffer ints
        cartcoords.addDouble(0.5); cartcoords.addDouble(pleft[0]); cartcoords.addDouble(pleft[1]);
        //    //flag that the object is detected
        cartcoords.addDouble(1.0);

        cartOutPort.write();
    }

    return true;

}

bool vTrackToRobotModule::updateModule()
{

    //if we haven't gazed for some time reset robot position
    static double htimeout = yarp::os::Time::now();
    if(yarp::os::Time::now() - htimeout > 5.0) {
        if(armdriver.isValid()) {
            arm->goToPose(armhomepos, armhomerot);
        }
        yarp::sig::Vector homefix(3);
        homefix[0] = -10; homefix[1] = 0; homefix[2] = 0.3;
        gazecontrol->lookAtFixationPoint(homefix);
        htimeout = yarp::os::Time::now();
    }

    //get the targets from the input ports
    yarp::sig::Vector leftTarget, rightTarget;
    inputPort.getTargets(leftTarget, rightTarget);

    //perform the type of control as specified
    bool gazePerformed = false;
    if(useDemoRedBall) {
        controlExternal(leftTarget, rightTarget);
        gazePerformed = true; //let the external function do all control
    } else {
        if(velocityControl)
            gazePerformed = controlVelocity(leftTarget, rightTarget);
        else
            gazePerformed = controlCartesian(leftTarget, rightTarget);
        if(usearm)
            controlArm(leftTarget, rightTarget);
    }

    //send out a debug if needed
    if(debugOutPort.getOutputCount()) {
        yarp::sig::Vector values(6);
        yarp::sig::Vector pleft = leftTarget.subVector(0, 1);
        yarp::sig::Vector pright = rightTarget.subVector(0, 1);
        yarp::sig::Vector xrobref;
        gazecontrol->triangulate3DPoint(pleft, pright, xrobref);
        values[0] = xrobref[0]; values[1] = xrobref[1]; values[2] = xrobref[2];
        values[0] = arm_target_position[0]; values[1] = arm_target_position[1]; values[2] = arm_target_position[2];
        if(armdriver.isValid()) {
            yarp::sig::Vector x_arm, o_arm;
            arm->getPose(x_arm, o_arm);
            values[3] = x_arm[0];
            values[4] = x_arm[1];
            values[5] = x_arm[2];
        }
        debugOutPort.prepare() = values;
        debugOutPort.write();
    }

    //reset the timeout if needed
    if(gazePerformed)
        htimeout = yarp::os::Time::now();

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

