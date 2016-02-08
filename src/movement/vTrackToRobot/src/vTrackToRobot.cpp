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

using namespace yarp::math;

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

vTrackToRobotManager::vTrackToRobotManager()
{

    method = fromgaze;
    gazecontrol = 0;

}

bool vTrackToRobotManager::setMethod(std::string methodname)
{
    if(methodname == "gaze") method = fromgaze;
    if(methodname == "size") method = fromsize;
    if(methodname == "stereo") method = fromstereo;

}

/******************************************************************************/
bool vTrackToRobotManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    if(!yarp::os::BufferedPort<emorph::vBottle>::open(inPortName)) {
        std::cerr << "Could not open: " << inPortName << std::endl;
        return false;
    }

    std::string outPortName = "/" + name + "/vCartOut:o";
    if(!cartOutPort.open(outPortName)) {
        std::cerr << "Could not open: " << outPortName << std::endl;
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


    if(method != fromgaze) return true;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + name);
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid()) gazedriver.view(gazecontrol);

    return gazedriver.isValid();
}

void vTrackToRobotManager::interrupt()
{
    std::cout << "Interrupting Manager" << std::endl;
    cartOutPort.interrupt();
    scopeOutPort.interrupt();
    positionOutPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    std::cout << "Interrupted Manager" << std::endl;
}

void vTrackToRobotManager::close()
{
    std::cout << "Closing Event Manager" << std::endl;

    gazecontrol->stopControl();
    gazedriver.close();
    cartOutPort.close();
    scopeOutPort.close();
    positionOutPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    std::cout << "Closed Event Manager" << std::endl;

}

/******************************************************************************/
void vTrackToRobotManager::onRead(emorph::vBottle &vBottleIn)
{

    //always print current position
    yarp::sig::Vector cpx(2), cx(3);
    cpx(0) = 64; cpx(1) = 64;
    gazecontrol->get3DPoint(0, cpx, 0.5, cx);

    //get the events and see if we can get a ball observation
    yarp::sig::Vector px(2), x(3); x = 0;
    emorph::vQueue q = vBottleIn.getSorted<emorph::ClusterEventGauss>();

    double eyez_mean = 0;
    if(q.size()) {

        emorph::ClusterEventGauss * v =
                q.back()->getAs<emorph::ClusterEventGauss>();

        px[0] += v->getYCog();
        px[1] += (127 - v->getXCog());
        double eyez = (-2.5 * v->getXSigma2() + 70)/100.0;

        recentgazelocs.push_back(px);
        recenteyezs.push_back(eyez);

        if(recentgazelocs.size() > 20) {
            recentgazelocs.pop_front();
            recenteyezs.pop_front();

            for(int i = 1; i < recenteyezs.size(); i++)
                eyez_mean += recenteyezs[i];
            eyez_mean /= recenteyezs.size();

            double x_mean = 0, y_mean = 0;
            for(int i = 1; i < recentgazelocs.size(); i++) {
                x_mean += recentgazelocs[i][0];
                y_mean += recentgazelocs[i][1];
            }
            x_mean /= recentgazelocs.size();
            y_mean /= recentgazelocs.size();

            bool gaze = false;
            if(x_mean - px[0] < 5 && y_mean - px[1] < 5)
                gaze = true;


            if(gaze) {

                //turn u/v into xyz
                gazecontrol->get3DPoint(0, px, eyez_mean, x);
                //std::cout << eyez_mean << std::endl;

                //and look there
                //std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
                gazecontrol->lookAtFixationPoint(x);
            }


        }

    }

    if(positionOutPort.getOutputCount() && eyez_mean) {
        yarp::os::Bottle &posdump = positionOutPort.prepare();
        posdump.clear();
        posdump.addDouble(eyez_mean);
        //posdump.addDouble(cx[0]); posdump.addDouble(cx[1]); posdump.addDouble(cx[2]);
        //posdump.addDouble(x[0]); posdump.addDouble(x[1]); posdump.addDouble(x[2]);
        yarp::os::Stamp st; this->getEnvelope(st);
        positionOutPort.setEnvelope(st);
        positionOutPort.write();
    }

    return;


    //this is the eye pose
    yarp::sig::Vector xeye,oeye;
    gazecontrol->getLeftEyePose(xeye,oeye);

    //this does the transformation
    yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
    T(0,3)=xeye[0];
    T(1,3)=xeye[1];
    T(2,3)=xeye[2];
    std::cout << "initial rotation matrix" << std::endl;
    std::cout << T.toString() << std::endl;

    std::cout << "initial translations" << std::endl;
    std::cout << xeye.toString() << std::endl;

    yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
    std::cout << "inverted rotation matrix" << std::endl;
    std::cout << Ti.toString() << std::endl;


    //this was the target in eye coordinates
    yarp::sig::Vector fp(4);
    fp[0]=x[0];
    fp[1]=x[1];
    fp[2]=x[2];
    fp[3]=1.0;

    std::cout << "Multiplied by" << std::endl;
    std::cout << fp.toString() << std::endl;




    yarp::sig::Vector tp=Ti*fp;
    std::cout << "Equals:" << std::endl;
    std::cout << tp.toString() << std::endl;

    //targetPos in the eye reference frame
    //std::cout << "2D point: " << px.toString() << std::endl;
    //std::cout << "3D point: " << x.toString() << std::endl;





//    if(!scopeOutPort.getOutputCount()) return;
//    yarp::os::Bottle &scopeBot = scopeOutPort.prepare();
//    scopeBot.clear();
//    scopeBot.addDouble(x[0]); scopeBot.addDouble(x[1]); scopeBot.addDouble(x[2]);
//    scopeBot.addDouble(px[0]); scopeBot.addDouble(px[1]);
//    scopeOutPort.write();

    //do some sanity checks on the xyz position so we don't break the robot (again)
    bool error = false;
    if(tp[0] < -0.3 || tp[0] > 0.3) error = true;
    if(tp[1] < -0.3 || tp[1] > 0.3) error = true;
    if(tp[2] < 0.49 || tp[2] > 0.51) error = true;

    if(error) {
        std::cout << "ERROR: position " << tp.toString() << std::endl;
        return;
    }

    //return;


//    yarp::sig::Vector x,o;
//    gazecontrol->getLeftEyePose(x,o);

//    Matrix T=yarp::sig::axis2dcm(o);
//    T(0,3)=x[0];
//    T(1,3)=x[1];
//    T(2,3)=x[2];

//    targetPos=T*fp;


    yarp::os::Bottle& BottleOut = cartOutPort.prepare();
    BottleOut.clear();
    //add the XYZ position
    BottleOut.add(tp[0]); BottleOut.add(tp[1]); BottleOut.add(tp[2]);
    //BottleOut.add(-1.0); BottleOut.add(0.0); BottleOut.add(-0.3);
    //add some buffer ints
    BottleOut.add(0.5); BottleOut.add(px[0]); BottleOut.add(px[1]);
    //flag that the object is detected
    BottleOut.add(1.0);

    //std::cout << "Bottle: " << BottleOut.toString() << std::endl;

    cartOutPort.write();

    //send some data for the scope if one is connected

}


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
            rf.check("method", yarp::os::Value("usegaze")).asString();

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




//empty line to make gcc happy
