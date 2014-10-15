// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * Public License fo
 r more details
 */

/**
 * @file oInteractorThread.cpp
 * @brief Implementation of the thread (see header oiThread.h)
 */

#include <iCub/ActionPrimitivesLayer3.h>
#include <iCub/objectInteractorThread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>
#include <iCub/iDyn/iDynTransform.h>

#include <string>
#include <deque>
#include <set>
#include <map>
#include <cstring>
#include <string>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;
using namespace std;

#define THRATE 10

/************************************************************************/
void ActionPrimitivesLayer3::init() {
    printf("initialisation in ACTIONPRIMITIVES3 \n");
    configuredLayer3 = false;
    ActionPrimitivesLayer2::init();
}


/************************************************************************/
void ActionPrimitivesLayer3::postReachCallback() {
    // call the main postReachCallback()
    ActionPrimitivesLayer2::postReachCallback();
}


/************************************************************************/
void ActionPrimitivesLayer3::run() {
    // call the main run()
    // the order does matter
    ActionPrimitivesLayer2::run();
}


/************************************************************************/
bool ActionPrimitivesLayer3::open(Property &opt) {
    printf("OPENING in ACTIONPRIMITIVESLAYER3 \n");
    if (!skipFatherPart) {
        ActionPrimitivesLayer2::open(opt);
        printf("not skipping the father part; opening layer2 ");
    }

    if (configuredLayer3)
    {
        printMessage("WARNING: already configured\n");
        return true;
    }

    if (configuredLayer2) {
        printf("openCHECK: configuredLayer2  \n");
        return configuredLayer3 = true;
    }
    else {
        printf("openCHECK: NOT configuredLayer2 \n");
        return false;
    }
}


/************************************************************************/
void ActionPrimitivesLayer3::alignJointsBounds() {
    ActionPrimitivesLayer2::alignJointsBounds();
}


/************************************************************************/

bool ActionPrimitivesLayer3::isValid() const {
    printf("configuredLayer3 %d \n", configuredLayer3);
    printf("configuredLayer2 %d \n", ActionPrimitivesLayer2::isValid());
    return (ActionPrimitivesLayer2::isValid() && configuredLayer3);
}


/************************************************************************/
void ActionPrimitivesLayer3::close()
{
    if (closed)
        return;
    printf ("Closing ActionPrimitivesLayer2 after Layer3 \n");
    // call the main close()
    // the order does matter
    ActionPrimitivesLayer2::close();
}

/************************************************************************/

bool ActionPrimitivesLayer3::grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
           const yarp::sig::Vector &d1, const yarp::sig::Vector &d2) {
               return ActionPrimitivesLayer2::grasp(x, o, d1, d2);
}

/************************************************************************/
bool ActionPrimitivesLayer3::grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
           const yarp::sig::Vector &d) {
               return ActionPrimitivesLayer2::grasp(x, o, d);
}

/************************************************************************/
bool ActionPrimitivesLayer3::touch(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
           const yarp::sig::Vector &d) {
               return ActionPrimitivesLayer2::touch(x, o, d);
}

/************************************************************************/
bool ActionPrimitivesLayer3::grasp2(const Vector &x, const Vector &o,
                                   const Vector &d1, const Vector &d2) {
    if (configured)
    {
        printMessage("start grasping\n");

        // latch the offset
        latchWrenchOffset();

        enableContactDetection();

        pushAction(x+d1,o,"open_hand");
        pushAction(x,o,ACTIONPRIM_DISABLE_EXECTIME,execLiftAndGrasp);
        // the remaining part is done in the callback

        // save data
        grasp_d2=d2;
        grasp_o=o;

        return true;
    }
    else
        return false;
}

/************************************************************************/
bool ActionPrimitivesLayer3::push(const Vector &x1, const Vector &o1,
                                 const Vector &x2, const Vector &o2,
                                 const double execTime) {
    if (configuredLayer3) {
        printMessage("start pushing in Layer3\n");
        disableTorsoDof();
        latchWrenchOffset();
        enableContactDetection();
        pushAction(x1,o1,"karate_hand");
        pushAction(x2,o2,execTime);
        pushAction(x1,o1);
        disableContactDetection();
        enableTorsoDof();
        return true;
    }
    else
        return false;
}

/************************************************************************/

bool ActionPrimitivesLayer3::tap(const Vector &x1, const Vector &o1,
                                 const Vector &x2, const Vector &o2,
                                 const double execTime)
{
    if (configured) {
        printMessage("start tapping of Layer3 \n");
        bool wrench = ActionPrimitivesLayer2::latchWrenchOffset();
        printf("wrench %d", wrench);
        bool enabled = ActionPrimitivesLayer2::enableContactDetection();
        printf("enabled %d", enabled);
        pushAction(x1,o1,"karate_hand");
        pushAction(x2,o2,ACTIONPRIM_DISABLE_EXECTIME);
        pushAction(x1,o1);
        ActionPrimitivesLayer2::disableContactDetection();
        return true;
    }
    else
        return false; 
}



/************************************************************************/
bool ActionPrimitivesLayer3::grasp2(const Vector &x, const Vector &o,
                                   const Vector &d) {
    return ActionPrimitivesLayer2::grasp(x,o,d);
}


/************************************************************************/
bool ActionPrimitivesLayer3::touch2(const Vector &x, const Vector &o, const Vector &d) {
    if (configured) {
        printMessage("start touching\n");

        // latch the offset
        latchWrenchOffset();

        enableContactDetection();

        pushAction(x+d,o,"karate_hand");
        pushAction(x,o,ACTIONPRIM_DISABLE_EXECTIME,execTouch);

        return true;
    }
    else
        return false;
}

/************************************************************************/
//bool ActionPrimitivesLayer3::latchWrenchOffset() {
//    return ActionPrimitivesLayer2::latchWrenchOffset();
//}


/************************************************************************/
bool ActionPrimitivesLayer3::getExtWrench(Vector &wrench) const {
    return ActionPrimitivesLayer2::getExtWrench(wrench);
}


/************************************************************************/
bool ActionPrimitivesLayer3::getExtForceThres(double &thres) const {
    return ActionPrimitivesLayer2::getExtForceThres(thres);
}


/************************************************************************/
bool ActionPrimitivesLayer3::setExtForceThres(const double thres) {
    return ActionPrimitivesLayer2::setExtForceThres(thres);
}


/************************************************************************/
//bool ActionPrimitivesLayer3::enableContactDetection() {
//    return ActionPrimitivesLayer2::enableContactDetection();
//}


/************************************************************************/
//bool ActionPrimitivesLayer3::disableContactDetection() {
//    return ActionPrimitivesLayer2::disableContactDetection();
//}


/************************************************************************/
bool ActionPrimitivesLayer3::isContactDetectionEnabled(bool &f) const {
    return ActionPrimitivesLayer2::isContactDetectionEnabled(f);
}


/************************************************************************/
bool ActionPrimitivesLayer3::checkContact(bool &f) const {
    return ActionPrimitivesLayer2::checkContact(f);
}


ActionPrimitivesLayer3::~ActionPrimitivesLayer3() {
    ActionPrimitivesLayer2::close();
}
