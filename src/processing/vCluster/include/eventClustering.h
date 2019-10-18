/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
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

#ifndef __ICUB_EVENTCLUSTERING_MOD_H__
#define __ICUB_EVENTCLUSTERING_MOD_H__

#include <yarp/os/all.h>
#include <event-driven/all.h>
#include <event-driven/deprecated.h>

#include "trackerPool.h"

#include <ctime>
#include <string>

/******************************************************************************/
//EventBottleManager
/******************************************************************************/
class EventBottleManager : public yarp::os::BufferedPort<ev::vBottle>
{
    private:

        yarp::os::BufferedPort<ev::vBottle>     outPort;            //output port for the eventBottle with the new events computed by the module

        //create trackers, left and right
        TrackerPool tracker_pool_left;
        TrackerPool tracker_pool_right;

    public:

        void setAllParameters(double alpha_shape, double alpha_pos,
                              double Tact, double Tinact, double Tfree,
                              double Tevent, double SigX, double SigY,
                              double SigXY, bool Fixedshape, int Regrate,
                              double Maxdist, double decay_tau,
                              double clusterLimit);

        bool    open(std::string moduleName);
        bool    init();
        void    close();
        void    onRead(ev::vBottle &bot);
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
