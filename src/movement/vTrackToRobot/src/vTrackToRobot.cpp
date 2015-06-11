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

    std::string outPortName = "/" + name + "/Bottle:o";
    if(!cartOutPort.open(outPortName)) {
        std::cerr << "Could not open: " << outPortName << std::endl;
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
    cartOutPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

void vTrackToRobotManager::close()
{
    cartOutPort.close();
    gazedriver.close();
    if(gazecontrol) delete gazecontrol; gazecontrol = 0;
    yarp::os::BufferedPort<emorph::vBottle>::close();

}

/******************************************************************************/
void vTrackToRobotManager::onRead(emorph::vBottle &vBottleIn)
{

    emorph::vQueue::reverse_iterator qi;
    emorph::vQueue q = vBottleIn.getSorted<emorph::ClusterEventGauss>();
    yarp::sig::Vector px(2), x(3);

    if(!q.size()) return;
    emorph::ClusterEventGauss * v =
            q.back()->getAs<emorph::ClusterEventGauss>();
    if(!v) return;

    px[0] = v->getXCog(); px[1] = v->getYCog();
    std::cout << "Pixel: " << px.toString() << std::endl;
    gazecontrol->get3DPoint(0, px, 1.0, x);
    std::cout << "2D point: " << px.toString() << std::endl;
    std::cout << "3D point: " << x.toString() << std::endl;
    //gazecontrol->lookAtFixationPoint(x);


    yarp::os::Bottle& BottleOut = cartOutPort.prepare();
    BottleOut.clear();
    //add the XYZ position
    BottleOut.add(x[0]); BottleOut.add(x[1]); BottleOut.add(x[2]);
    //BottleOut.add(0.5); BottleOut.add(0.8); BottleOut.add(0.4);
    //add some buffer ints
    BottleOut.add(0.0); BottleOut.add(0.0); BottleOut.add(0.0);
    //flag that the object is detected
    BottleOut.add(1.0);

    std::cout << "Bottle: " << BottleOut.toString() << std::endl;

    cartOutPort.write();

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
