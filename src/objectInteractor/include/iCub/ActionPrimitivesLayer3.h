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
 * Public License for more details
 */

/**
 * @file actionPrimitivesLayer3.h
 * @brief Definition of a thread that allows the robot to perform third generation of actions
 * (see graspThread.cpp).
 */

#ifndef _ACTIONPRIMITIVES_LAYER3_H_
#define _ACTIONPRIMITIVES_LAYER3_H_

#include <iCub/ctrl/ctrlMath.h>
#include <iCub/action/actionPrimitives.h>


/**
* A derived class defining a second abstraction layer on top of 
* @ref ActionPrimitivesLayer2 father class. 
*  
* It internally extends the class of action performed using force sensor
*  
* @note Given as an example of how primitive actions can be 
*       combined in higher level actions, thus how further
*       layers can be inherited from the base class.
*/
class ActionPrimitivesLayer3 : public iCub::action::ActionPrimitivesLayer2 {
protected:
    bool configuredLayer3;
    virtual void init();
    virtual void alignJointsBounds();
    virtual void postReachCallback();
    virtual void run();
public:
    /**
    * Default Constructor. 
    */
    ActionPrimitivesLayer3() : ActionPrimitivesLayer2() {printf("instantiate general class"); };

    /**
    * Constructor. 
    * @param opt the Property used to configure the object after its
    *            creation.
    */
    ActionPrimitivesLayer3(yarp::os::Property &opt)  { printf("instantiate general class \n");
        ActionPrimitivesLayer2::init();
        printf("init terminated \n");
        skipFatherPart=false;
        ActionPrimitivesLayer2::open(opt);
        printf("opening the interface \n");
    } ;

    /**
    * Destructor. 
    *  
    * @note it calls the close() method. 
    */
    virtual ~ActionPrimitivesLayer3();

    /**
    * Configure the object.
    * @param opt the Property used to configure the object after its
    *            creation.
    *  
    * @note To be called after object creation. 
    *  
    * Further available options are: 
    *  
    * @b ext_force_thres <double>: specify the maximum external 
    *    force magnitude applied to the end-effector in order to
    *    detect contact between end-effector and objects while
    *    reaching.
    *  
    * @note A port called <i> /<local>/<part>/ft:i </i> is open to
    *       acquire data provided by \ref force/torque sensor.
    */
    virtual bool open(yarp::os::Property &opt);

    /**
    * Check if the object is initialized correctly. 
    * @return true/fail on success/fail. 
    */
    virtual bool isValid() const;

    /**
    * Deallocate the object.
    */
    virtual void close();

    /**
    * More evolute version of grasp. It exploits the contact 
    * detection in order to lift up a bit the hand prior to 
    * grasping (this happens only after contact).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d1 the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @param d2 the displacement [m] that identifies the amount of 
    *           the lift after the contact.
    * @return true/false on success/fail. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d1, const yarp::sig::Vector &d2);

    /**
    * The usual grasp is still available.
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @return true/false on success/fail. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * More evolute version of touch, exploiting contact detection.
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/touching
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          touching.
    * @return true/false on success/fail. 
    */
    virtual bool touch(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Latch the wrench to compensate for offset in the sensor 
    * measures. 
    * @return true/false on success/fail. 
    */
    virtual bool latchWrenchOffset();

    /**
    * Retrieve the current wrench on the end-effector.
    * @param wrench a vector containing the external forces/moments 
    *               acting on the end-effector.
    * @return true/false on success/fail. 
    */
    virtual bool getExtWrench(yarp::sig::Vector &wrench) const;

    /**
    * Retrieve the current threshold on the external force used to 
    * stop the limb while reaching. 
    * @param thres where to return the threshold.
    * @return true/false on success/fail. 
    */
    virtual bool getExtForceThres(double &thres) const;

    /**
    * Set the threshold on the external force used to stop the limb
    * while reaching. 
    * @param thres the new threshold.
    * @return true/false on success/fail. 
    */
    virtual bool setExtForceThres(const double thres);

    /**
    * Self-explaining :)
    * @return true/false on success/fail. 
    */
    virtual bool enableContactDetection();

    /**
    * Self-explaining :)
    * @return true/false on success/fail. 
    */
    virtual bool disableContactDetection();

    /**
    * Self-explaining :) 
    * @param f the result of the check.  
    * @return true/false on success/fail. 
    */
    virtual bool isContactDetectionEnabled(bool &f) const;

    /**
    * Check whether the reaching has been stopped due to a contact 
    * with external objects. 
    * @param f the result of the check. 
    * @return true/false on success/fail. 
    */
    virtual bool checkContact(bool &f) const;

        /**
    * More evolute version of grasp. It exploits the contact 
    * detection in order to lift up a bit the hand prior to 
    * grasping (this happens only after contact).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d1 the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @param d2 the displacement [m] that identifies the amount of 
    *           the lift after the contact.
    * @return true/false on success/fail. 
    */
    virtual bool grasp2(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d1, const yarp::sig::Vector &d2);

    /**
    * Grasp the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ... 
    * pushAction(x+d,o,"open_hand"); 
    * pushAction(x,o); 
    * pushAction("close_hand"); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x+d,o) opening the hand, then reachs for (x,o)
    * and finally closes the hand. 
    */
    bool grasp2(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Touch the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/touching
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          touching.
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ... 
    * pushAction(x+d,o,"karate_hand"); 
    * pushAction(x,o); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x+d,o), then reachs for (x,o). 
    * Similar to grasp but without final hand action. 
    */
    bool touch2(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * push the given target (combined action).
    * @param x1 the fisrt 3-d target position [m]. 
    * @param o1 the first 4-d hand orientation (given in axis-angle 
    *           representation: ax ay az angle in rad).
    * @param x2 the second 3-d target position [m]. 
    * @param o2 the second 4-d hand orientation (given in axis-angle
    *           representation: ax ay az angle in rad).
    * @param execTime the arm action execution time only while 
    *          pushing [s] (to be specified iff different from
    *          default value).
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ...
    * pushAction(x1,o1,"karate_hand");
    * pushAction(x2,o2,execTime);
    * pushAction(x1,o1); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x1,o1), then reachs for (x2,o2) and then again
    * for (x1,o1).
    */
    bool push(const yarp::sig::Vector &x1, const yarp::sig::Vector &o1,
                     const yarp::sig::Vector &x2, const yarp::sig::Vector &o2,
                     const double execTime=ACTIONPRIM_DISABLE_EXECTIME);
};

#endif  //_ACTIONPRIMITIVES_LAYER3_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
