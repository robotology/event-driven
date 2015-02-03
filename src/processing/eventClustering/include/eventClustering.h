/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi and Arren Glover
 * email:  chiara.bartolozzi@iit.it, arren.glover@iit.it
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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/os/RpcClient.h>

#include <iCub/emorph/all.h>
//#include <iCub/emorph/eventCodec.h>

#include "trackerPool.h"

#include <time.h>
#include <string>

/******************************************************************************/
//EventBottleManager
/******************************************************************************/
class EventBottleManager : public yarp::os::BufferedPort<emorph::vBottle>
{
    private:

        yarp::os::BufferedPort<emorph::vBottle>     outPort;            //output port for the eventBottle with the new events computed by the module

        //create trackers, left and right
        TrackerPool tracker_pool_left;
        TrackerPool tracker_pool_right;
    
    public:
    
        void setAllParameters(double alpha_shape, double alpha_pos,
                         double Tact, double Tinact, double Tfree,
                         double Tevent, double SigX, double SigY,
                         double SigXY, bool Fixedshape, int Regrate,
                         double Maxdist, double decay_tau);

        bool    open(std::string moduleName);
        bool    init();
        void    close();
        void    onRead(emorph::vBottle &bot);
        void    interrupt();
  
};

/******************************************************************************/
//EventClustering
/******************************************************************************/
class EventClustering:public yarp::os::RFModule
{
    /* module parameters */
    yarp::os::RpcServer     rpcPort;

    /* pointer to a new manager */
    EventBottleManager      eventBottleManager;
    bool                    closing;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();

};


#endif
//empty line to make gcc happy
