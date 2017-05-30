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
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

//vTrackToRobotManager::vTrackToRobotManager()
//{

//    demo = gazedemo;
//    gazecontrol = 0;
//    p_eyez = 0.5;
//    gazingActive = false;
//    //inital gaze
//    xrobref.resize(3);
//    xrobref[0]=-0.4; //x = -0.4 (distance infront -ive)
//    xrobref[1]=0; //y = 0 (left-right)
//    xrobref[2]=0.3; //z = 0.3 (up/down)
//    leftTarget.resize(3);
//    rightTarget.resize(3);

//    Ythresh = 20;
//    Rthresh = 5;

//}

//void vTrackToRobotManager::setDemo(std::string demoname)
//{
//    if(demoname == "gaze") demo = gazedemo;
//    if(demoname == "grasp") demo = graspdemo;

//}

/******************************************************************************/
//bool vTrackToRobotManager::open(const std::string &name)
//{
//    //and open the input port

//    this->useCallback();

//    if(!yarp::os::BufferedPort<ev::vBottle>::open(name + "/vBottle:i"))
//        return false;

//    if(!cartOutPort.open(name + "/vCartOut:o"))
//        return false;

//    if(!scopeOutPort.open(name + "/scope:o"))
//        return false;

//    if(!positionOutPort.open(name + "/posdump:o"))
//        return false;

//    yarp::os::Property options;
//    options.put("device", "gazecontrollerclient");
//    options.put("local", name);
//    options.put("remote", "/iKinGazeCtrl");
//    gazedriver.open(options);
//    if(gazedriver.isValid())
//        gazedriver.view(gazecontrol);
//    else
//        yWarning() << "Gaze Driver not opened and will not be used";

//    return true;
//}

//void vTrackToRobotManager::interrupt()
//{
//    std::cout << "Interrupting Manager" << std::endl;
//    cartOutPort.interrupt();
//    scopeOutPort.interrupt();
//    positionOutPort.interrupt();
//    yarp::os::BufferedPort<ev::vBottle>::interrupt();
//    std::cout << "Interrupted Manager" << std::endl;
//}

//void vTrackToRobotManager::close()
//{
//    std::cout << "Closing Event Manager" << std::endl;

//    if(gazedriver.isValid()) {
//        gazecontrol->stopControl();
//        gazedriver.close();
//    }
//    cartOutPort.close();
//    scopeOutPort.close();
//    positionOutPort.close();
//    yarp::os::BufferedPort<ev::vBottle>::close();

//    std::cout << "Closed Event Manager" << std::endl;

//}

/******************************************************************************/
//void vTrackToRobotManager::onRead(ev::vBottle &vBottleIn)
//{

//    //get the yarpstamp
//    yarp::os::Stamp st;
//    this->getEnvelope(st);

//    //get the Q
//    vQueue q = vBottleIn.get<GaussianAE>();
//    if(q.empty()) {
//        yWarning() << "q empty in callback function?";
//        return;
//    }

//    //update our current best position of the ball in both cameras
//    bool leftupdated = false, rightupdated = false;
//    vQueue::reverse_iterator qi = q.rbegin();
//    while((!leftupdated || !rightupdated) && qi != q.rend()) {
//        auto v = is_event<GaussianAE>(*qi);
//        if(v->channel == VRIGHT && !rightupdated) {
//            rightupdated = true;
//            rightTarget[0] = v->x;
//            rightTarget[1] = v->y;
//            rightTarget[2] = v->sigx; //radius
//        } else if(v->channel == VLEFT && !leftupdated) {
//            leftupdated = true;
//            leftTarget[0] = v->x;
//            leftTarget[1] = v->y;
//            leftTarget[2] = v->sigx; //radius
//        }
//        qi++;
//    }




//    return;





//    bool dogaze = true;
//    auto vc = is_event<GaussianAE>(q.back());
//    int bestts = vc->stamp;

////    px[0] = 303 - vc->x;
////    px[1] = 239 - vc->y;
//    //turn u/v into xyz
//    if(gazedriver.isValid()) {
//        //gazecontrol->get3DPoint(0, px, (-2.4 * p_eyez + 70)/100.0, xrobref);
//        double zpos = -0.02 * p_eyez + 0.8;
//        zpos = std::min(zpos, 0.5);
//        zpos = std::max(zpos, 0.3);

//        gazecontrol->get3DPoint(1, px, zpos, xrobref);

//        std::cout << px.toString() << " " << xrobref.toString() << std::endl;
//    }




    //DO GAZE
    //find the median position in xyz space and gaze there
    //yarp::sig::Vector px(2);        //pixel in uv
    //yarp::sig::Vector x(3); x = 0;  //position in xyz (iCub ref frame)
    //px[0] = medy;
    //px[1] = 127 - medx;
//    if(gazedriver.isValid()) {

//        //if we use the gaze controller to gaze then go ahead
//        bool motionDone = false;
//        gazecontrol->checkMotionDone(&motionDone);
//        if(demo == gazedemo && gazingActive) {
//            gazecontrol->lookAtFixationPoint(xrobref);
//        }
//    }

//    return;




//    //DUMP POSITIONS
//    //find the position of the eyes in the current position
//    yarp::sig::Vector cpx(2); cpx[0] = 64; cpx[1] = 64;
//    yarp::sig::Vector cx(3); cx = 0;  //position in xyz (eye ref frame)
//    if(gazedriver.isValid()) {
//        gazecontrol->get3DPoint(0, cpx, (-2.4 * p_eyez + 70)/100.0, cx);
//    }
//    if(positionOutPort.getOutputCount()) {
//        yarp::os::Bottle &posdump = positionOutPort.prepare();
//        posdump.clear();
//        posdump.addInt(bestts);
//        posdump.addDouble(cx[0]); posdump.addDouble(cx[1]); posdump.addDouble(cx[2]);
//        posdump.addDouble(xrobref[0]); posdump.addDouble(xrobref[1]); posdump.addDouble(xrobref[2]);
//        positionOutPort.setEnvelope(st);
//        positionOutPort.write();
//    }

//    return;

//}
//void vTrackToRobotManager::onRead(eventdriven::vBottle &vBottleIn)
//{

//    //always print current position
//    yarp::sig::Vector cpx(2), cx(3);
//    cpx(0) = 64; cpx(1) = 64;
//    gazecontrol->get3DPoint(0, cpx, p_eyez, cx);

//    //get the events and see if we can get a ball observation
//    yarp::sig::Vector px(2), x(3); x = 0;
//    eventdriven::vQueue q = vBottleIn.getSorted<eventdriven::ClusterEventGauss>();
//    eventdriven::vQueue qforts = vBottleIn.getAll();
//    qforts.wrapSort();


//    if(q.size()) {

//        eventdriven::ClusterEventGauss * v =
//                q.back()->getAs<eventdriven::ClusterEventGauss>();

//        px[0] += v->getYCog();
//        px[1] += (127 - v->getXCog());
//        double eyez = (-2.5 * v->getXSigma2() + 70)/100.0;

//        recentgazelocs.push_back(px);
//        recenteyezs.push_back(eyez);

//        if(recentgazelocs.size() > 20) {
//            recentgazelocs.pop_front();
//            recenteyezs.pop_front();

//            double eyez_mean = 0;
//            for(int i = 1; i < recenteyezs.size(); i++)
//                eyez_mean += recenteyezs[i];
//            eyez_mean /= recenteyezs.size();

//            double x_mean = 0, y_mean = 0;
//            for(int i = 1; i < recentgazelocs.size(); i++) {
//                x_mean += recentgazelocs[i][0];
//                y_mean += recentgazelocs[i][1];
//            }
//            x_mean /= recentgazelocs.size();
//            y_mean /= recentgazelocs.size();

//            bool gaze = false;
//            if(x_mean - px[0] < 5 && y_mean - px[1] < 5)
//                gaze = true;


//            if(gaze) {

//                //turn u/v into xyz
//                gazecontrol->get3DPoint(0, px, eyez_mean, x);
//                p_eyez = eyez_mean;
//                //std::cout << eyez_mean << std::endl;

//                //and look there
//                //std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
//                if(gazingActive)
//                    gazecontrol->lookAtFixationPoint(x);
//            }


//        }

//    }

//    if(positionOutPort.getOutputCount()) {
//        yarp::os::Bottle &posdump = positionOutPort.prepare();
//        posdump.clear();
//        posdump.addInt(qforts.front()->stamp);
//        posdump.addDouble(cx[0]); posdump.addDouble(cx[1]); posdump.addDouble(cx[2]);
//        posdump.addDouble(x[0]); posdump.addDouble(x[1]); posdump.addDouble(x[2]);
//        yarp::os::Stamp st; this->getEnvelope(st);
//        positionOutPort.setEnvelope(st);
//        positionOutPort.write();
//    }

//    return;


//    //this is the eye pose
//    yarp::sig::Vector xeye,oeye;
//    gazecontrol->getLeftEyePose(xeye,oeye);

//    //this does the transformation
//    yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
//    T(0,3)=xeye[0];
//    T(1,3)=xeye[1];
//    T(2,3)=xeye[2];
//    std::cout << "initial rotation matrix" << std::endl;
//    std::cout << T.toString() << std::endl;

//    std::cout << "initial translations" << std::endl;
//    std::cout << xeye.toString() << std::endl;

//    yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
//    std::cout << "inverted rotation matrix" << std::endl;
//    std::cout << Ti.toString() << std::endl;


//    //this was the target in eye coordinates
//    yarp::sig::Vector fp(4);
//    fp[0]=x[0];
//    fp[1]=x[1];
//    fp[2]=x[2];
//    fp[3]=1.0;

//    std::cout << "Multiplied by" << std::endl;
//    std::cout << fp.toString() << std::endl;




//    yarp::sig::Vector tp=Ti*fp;
//    std::cout << "Equals:" << std::endl;
//    std::cout << tp.toString() << std::endl;

//    //targetPos in the eye reference frame
//    //std::cout << "2D point: " << px.toString() << std::endl;
//    //std::cout << "3D point: " << x.toString() << std::endl;





////    if(!scopeOutPort.getOutputCount()) return;
////    yarp::os::Bottle &scopeBot = scopeOutPort.prepare();
////    scopeBot.clear();
////    scopeBot.addDouble(x[0]); scopeBot.addDouble(x[1]); scopeBot.addDouble(x[2]);
////    scopeBot.addDouble(px[0]); scopeBot.addDouble(px[1]);
////    scopeOutPort.write();

//    //do some sanity checks on the xyz position so we don't break the robot (again)
//    bool error = false;
//    if(tp[0] < -0.3 || tp[0] > 0.3) error = true;
//    if(tp[1] < -0.3 || tp[1] > 0.3) error = true;
//    if(tp[2] < 0.49 || tp[2] > 0.51) error = true;

//    if(error) {
//        std::cout << "ERROR: position " << tp.toString() << std::endl;
//        return;
//    }

//    //return;


////    yarp::sig::Vector x,o;
////    gazecontrol->getLeftEyePose(x,o);

////    Matrix T=yarp::sig::axis2dcm(o);
////    T(0,3)=x[0];
////    T(1,3)=x[1];
////    T(2,3)=x[2];

////    targetPos=T*fp;


//    yarp::os::Bottle& BottleOut = cartOutPort.prepare();
//    BottleOut.clear();
//    //add the XYZ position
//    BottleOut.add(tp[0]); BottleOut.add(tp[1]); BottleOut.add(tp[2]);
//    //BottleOut.add(-1.0); BottleOut.add(0.0); BottleOut.add(-0.3);
//    //add some buffer ints
//    BottleOut.add(0.5); BottleOut.add(px[0]); BottleOut.add(px[1]);
//    //flag that the object is detected
//    BottleOut.add(1.0);

//    //std::cout << "Bottle: " << BottleOut.toString() << std::endl;

//    cartOutPort.write();

//    //send some data for the scope if one is connected

//}


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
    period = rf.check("period", yarp::os::Value(0.1)).asDouble();
    gazingActive = rf.check("start", yarp::os::Value(false)).asBool();

    if(!rpcPort.open(getName() + "/control"))
        return false;
    this->attach(rpcPort);

    //inputPort.setDemo(rf.check("demo", yarp::os::Value("gaze")).asString());

    if(!inputPort.open(getName() + "/vBottle:i"))
        return false;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", getName());
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else
        yWarning() << "Gaze Driver not opened and will not be used";

    return true ;
}

bool vTrackToRobotModule::updateModule()
{
    yarp::sig::Vector leftTarget, rightTarget;
    inputPort.getTargets(leftTarget, rightTarget);

    //do our stereo target check
    if(std::abs(rightTarget[1] - leftTarget[1]) > yThresh) {
        yWarning() << "Y values not consistent for target";
        return true;
    }
    if(std::abs(rightTarget[2] - leftTarget[2]) > rThresh) {
        yWarning() << "Radius not consistent for target";
        return true;
    }

    if(!gazingActive) {
        yInfo() << "Gaze valid (gazing blocked)";
        return true;
    }

    if(gazedriver.isValid()) {

        yInfo() << "Doing gaze";

        yarp::sig::Vector pleft = leftTarget.subVector(0, 1);
        pleft[0] = 303 - pleft[0];
        pleft[1] = 239 - pleft[1];

        yarp::sig::Vector pright = rightTarget.subVector(0, 1);
        pright[0] = 303 - pright[0];
        pright[1] = 239 - pright[1];

        gazecontrol->lookAtStereoPixels(pleft, pright);
    } else {
        yInfo() << "Gaze valid (gazedriver invalid)";
    }

    return true;
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
    inputPort.interrupt();
    return yarp::os::RFModule::interruptModule();
}

bool vTrackToRobotModule::close()
{
    if(gazedriver.isValid()) {
        gazecontrol->stopControl();
        gazedriver.close();
    }
    inputPort.close();
    return yarp::os::RFModule::close();
}

