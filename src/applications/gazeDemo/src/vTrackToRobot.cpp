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

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

vTrackToRobotManager::vTrackToRobotManager()
{

    method = fromgaze;
    demo = gazedemo;
    gazecontrol = 0;
    p_eyez = 0.5;
    gazingActive = true;
    //inital gaze
    xrobref.resize(3);
    xrobref[0]=-0.4; //x = -0.4 (distance infront -ive)
    xrobref[1]=0; //y = 0 (left-right)
    xrobref[2]=0.3; //z = 0.3 (up/down)
    px.resize(2);
    medx = 64;
    medy = 64;
    px[0] = medy;
    px[1] = 127 - medx;
    lastdogazetime = 0;

    FIFO = ev::temporalSurface(340, 340);
    FIFO.setTemporalSize(250000 * 7.8125);

}

void vTrackToRobotManager::setMethod(std::string methodname)
{
    if(methodname == "gaze") method = fromgaze;
    if(methodname == "size") method = fromsize;
    if(methodname == "stereo") method = fromstereo;
}

void vTrackToRobotManager::setDemo(std::string demoname)
{
    if(demoname == "gaze") demo = gazedemo;
    if(demoname == "grasp") demo = graspdemo;

}

/******************************************************************************/
bool vTrackToRobotManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string vInPortName = "/" + name + "/vBottle:i";
    if(!yarp::os::BufferedPort<ev::vBottle>::open(vInPortName)) {
        std::cerr << "Could not open: " << vInPortName << std::endl;
        return false;
    }
    std::string vOutPortName = "/" + name + "/vBottle:o";
    if(!eventsOutPort.open(vOutPortName)) {
        std::cerr << "Could not open: " << vOutPortName << std::endl;
        return false;
    }

    std::string cartPortName = "/" + name + "/vCartOut:o";
    if(!cartOutPort.open(cartPortName)) {
        std::cerr << "Could not open: " << cartPortName << std::endl;
        return false;
    }

    std::string scopePortName = "/" + name + "/scope:o";
    if(!scopeOutPort.open(scopePortName)) {
        std::cerr << "Could not open: " << scopePortName << std::endl;
        return false;
    }

    std::string positionPortName = "/" + name + "/posdump:o";
    if(!positionOutPort.open(positionPortName)) {
        std::cerr << "Could not open: " << positionPortName << std::endl;
        return false;
    }


    //if(method != fromgaze) return true;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + name);
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else
        std::cerr << "Gaze Driver not opened and will not be used" << std::endl;

    return true;
}

void vTrackToRobotManager::interrupt()
{
    std::cout << "Interrupting Manager" << std::endl;
    eventsOutPort.interrupt();
    cartOutPort.interrupt();
    scopeOutPort.interrupt();
    positionOutPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
    std::cout << "Interrupted Manager" << std::endl;
}

void vTrackToRobotManager::close()
{
    std::cout << "Closing Event Manager" << std::endl;

    if(gazedriver.isValid()) {
        gazecontrol->stopControl();
        gazedriver.close();
    }
    eventsOutPort.close();
    cartOutPort.close();
    scopeOutPort.close();
    positionOutPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    std::cout << "Closed Event Manager" << std::endl;

}

/******************************************************************************/
void vTrackToRobotManager::onRead(ev::vBottle &vBottleIn)
{

    yarp::os::Stamp st;
    this->getEnvelope(st);

    //we just need to get our updated TS
    ev::vQueue q = vBottleIn.getAllSorted();
    if(q.empty()) return;
    int bestts = q.back()->stamp;
    FIFO.removeEvents(q.back());

    //get the events and see if we can get a ball observation
    //q = vBottleIn.getSorted<ev::ClusterEventGauss>();
    q = vBottleIn.get<GaussianAE>();

    bool dogaze;
    if(yarp::os::Time::now() > lastdogazetime + 3)
        dogaze = false;
    else
        dogaze = true;

    if(q.size()) {

        auto vc = is_event<GaussianAE>(q.back());

        //get the stamp
        bestts = vc->stamp;

        //get radius
        //p_eyez = (-2.5 * vc->getXSigma2() + 70)/100.0;
        p_eyez = vc->sigxy;
        //p_eyez = std::min(p_eyez, 16.0);

        //update our window
//        ev::event<ev::AddressEvent> v = ev::event<ev::AddressEvent>(new ev::AddressEvent());
//        v->setChannel(vc->getChannel());
//        v->setPolarity(vc->getChannel());
//        v->setX(vc->getXCog());
//        v->setY(vc->getYCog());
//        FIFO.addEvent(v);

//        //and then get everything in the current window
//        q = FIFO.getSurf();
//        n = q.size();

//        //compute the median
//        std::vector<int> xs, ys;
//        xs.resize(n); ys.resize(n);
//        for(int i = 0; i < n; i++) {
//            ev::event<ev::AddressEvent> vtw = ev::as_event<ev::AddressEvent>(q[i]);
//            xs[i] = 303 - vtw->x;
//            ys[i] = 239 - vtw->y;
//            //p_eyez = std::max(p_eyez, (double)vtw->getXSigma2());
//            //p_eyez = std::min(p_eyez, 16.0);
//        }

//        std::sort(xs.begin(), xs.end());
//        std::sort(ys.begin(), ys.end());

//        medx = xs[n / 2];
//        medy = ys[n / 2];

        //do error check for too much noise
//        double medstdx = 0, medstdy = 0;
//        for(int i = 0; i < n; i++) {
//            medstdx += pow(xs[i] - medx, 2.0);
//            medstdy += pow(ys[i] - medy, 2.0);
//        }
//        medstdx = sqrt(medstdx / n);
//        medstdy = sqrt(medstdy / n);

        //if(std::abs(vc->getXCog() - medx) < medstdx && std::abs(vc->getYCog() - medy) < medstdy) {
            //std::cout << "current observation within 1 std" << std::endl;
            //std::cout << medstdx << " " << medstdy << " " << n << std::endl;
            //if(medstdx < 10 && medstdy < 10 && n > 5) {
                dogaze = true;
                lastdogazetime = yarp::os::Time::now();
                px[0] = 303 - vc->x;
                px[1] = 239 - vc->y;
                //turn u/v into xyz
                if(gazedriver.isValid()) {
                    //gazecontrol->get3DPoint(0, px, (-2.4 * p_eyez + 70)/100.0, xrobref);
                    double zpos = -0.02 * p_eyez + 0.8;
                    zpos = std::min(zpos, 0.5);
                    zpos = std::max(zpos, 0.3);

                    gazecontrol->get3DPoint(1, px, zpos, xrobref);
                    std::cout << px.toString() << " " << xrobref.toString() << std::endl;
                }
            //}
        //}

    }


    //DO GAZE
    //find the median position in xyz space and gaze there
    //yarp::sig::Vector px(2);        //pixel in uv
    //yarp::sig::Vector x(3); x = 0;  //position in xyz (iCub ref frame)
    //px[0] = medy;
    //px[1] = 127 - medx;
    if(gazedriver.isValid() && dogaze) {

        //if we use the gaze controller to gaze then go ahead
        if(demo == gazedemo && gazingActive)
            gazecontrol->lookAtFixationPoint(xrobref);
    }

    if(gazedriver.isValid() && dogaze && demo == graspdemo && gazingActive) {
    //if(gazedriver.isValid() && demo == graspdemo && gazingActive) {

        //this is the eye pose
        yarp::sig::Vector xeye,oeye;
        gazecontrol->getLeftEyePose(xeye,oeye);

        //this does the transformation
        yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
        T(0,3)=xeye[0];
        T(1,3)=xeye[1];
        T(2,3)=xeye[2];
        //std::cout << "initial rotation matrix" << std::endl;
        //std::cout << T.toString() << std::endl;

        //std::cout << "initial translations" << std::endl;
        //std::cout << xeye.toString() << std::endl;

        yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
        //std::cout << "inverted rotation matrix" << std::endl;
        //std::cout << Ti.toString() << std::endl;


        //this was the target in eye coordinates
        yarp::sig::Vector fp(4);
        fp[0]=xrobref[0];
        fp[1]=xrobref[1];
        fp[2]=xrobref[2];
        fp[3]=1.0;

        //std::cout << "Multiplied by" << std::endl;
        //std::cout << fp.toString() << std::endl;


        yarp::sig::Vector tp=Ti*fp;
        //std::cout << "Equals:" << std::endl;
        //std::cout << tp.toString() << std::endl;
        if(cartOutPort.getOutputCount()) {
            yarp::os::Bottle &cartcoords = cartOutPort.prepare();
            cartcoords.clear();
            //    //add the XYZ position
            cartcoords.add(tp[0]); cartcoords.add(tp[1]); cartcoords.add(tp[2]);
            //cartcoords.add(-1.0); cartcoords.add(0.0); cartcoords.add(-0.3);
            //    //add some buffer ints
            cartcoords.add(0.5); cartcoords.add(px[0]); cartcoords.add(px[1]);
            //    //flag that the object is detected
            cartcoords.add(1.0);

            //std::cout << "Bottle: " << cartcoords.toString() << std::endl;
            //targetPos in the eye reference frame
            //std::cout << "2D point: " << px.toString() << std::endl;
            //std::cout << "3D point: " << x.toString() << std::endl;
            cartOutPort.write();
        }

    }


    //DUMP POSITIONS
    //find the position of the eyes in the current position
    yarp::sig::Vector cpx(2); cpx[0] = 64; cpx[1] = 64;
    yarp::sig::Vector cx(3); cx = 0;  //position in xyz (eye ref frame)
    if(gazedriver.isValid()) {
        gazecontrol->get3DPoint(0, cpx, (-2.4 * p_eyez + 70)/100.0, cx);
    }
    if(positionOutPort.getOutputCount()) {
        yarp::os::Bottle &posdump = positionOutPort.prepare();
        posdump.clear();
        posdump.addInt(bestts);
        posdump.addDouble(cx[0]); posdump.addDouble(cx[1]); posdump.addDouble(cx[2]);
        posdump.addDouble(xrobref[0]); posdump.addDouble(xrobref[1]); posdump.addDouble(xrobref[2]);
        positionOutPort.setEnvelope(st);
        positionOutPort.write();
    }


    //PASS THROUGH EVENTS
    if(eventsOutPort.getOutputCount()) {
        //add all the address and flow events
        ev::vBottle &vBottleOut = eventsOutPort.prepare();
        vBottleOut = vBottleIn;

        //add the gaze point event
        if(dogaze) {
            auto circevent = make_event<GaussianAE>();
            circevent->stamp = bestts;
            circevent->setChannel(0);
            circevent->x = (int)medx;
            circevent->y = (int)medy;
            circevent->sigx = (int)p_eyez;
            circevent->sigy = 1;
            circevent->ID = 1;
            vBottleOut.addEvent(circevent);

        }

        //write the output
        eventsOutPort.setEnvelope(st);
        eventsOutPort.write();
    }





    return;

}
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

bool vTrackToRobotModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vTrackToRobot")).asString();
    setName(moduleName.c_str());

    std::string method =
            rf.check("method", yarp::os::Value("gaze")).asString();

    std::string rpcportname = "/" + moduleName + "/control";
    if(!rpcPort.open(rpcportname)) {
        std::cerr << "Could not open RPC port" << std::endl;
    }
    this->attach(rpcPort);

    vTrackToRobot.setDemo(rf.check("demo", yarp::os::Value("gaze")).asString());

    vTrackToRobot.setMethod(method);
    if(!vTrackToRobot.open(moduleName)) {
        std::cerr << "Could Not Open vTrackToRobotModule" << std::endl;
        return false;
    }

    return true ;
}

/******************************************************************************/
bool vTrackToRobotModule::interruptModule()
{
    vTrackToRobot.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vTrackToRobotModule::close()
{
    vTrackToRobot.close();
    yarp::os::RFModule::close();
    return true;
}

/******************************************************************************/
bool vTrackToRobotModule::updateModule()
{
    return true;
}

/******************************************************************************/
double vTrackToRobotModule::getPeriod()
{
    return 1;
}

bool vTrackToRobotModule::respond(const yarp::os::Bottle &command,
                                  yarp::os::Bottle &reply)
{
    reply.clear();

    if(command.get(0).asString() == "start") {
        reply.addString("starting");
        this->vTrackToRobot.startGazing();
    } else if(command.get(0).asString() == "stop") {
        reply.addString("stopping");
        this->vTrackToRobot.stopGazing();
    } else {
        return false;
    }

    return true;


}



//empty line to make gcc happy
