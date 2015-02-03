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

#include "eventClustering.h"

/******************************************************************************/
//EventClustering
/******************************************************************************/
bool EventClustering::configure(yarp::os::ResourceFinder &rf)
{
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("eventClustering"),
                                      "module name (string)").asString();

    setName(moduleName.c_str());

    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << rpcPortName << " : Unable to open port" << std::endl;
        return false;
    }
    attach(rpcPort);


    double alphaShape = rf.check("alpha_shape",
                        yarp::os::Value(0.01),
                        "alpha shape (double)").asDouble();

    double alphaPos = rf.check("alpha_pos",
                        yarp::os::Value(0.1),
                        "alpha pos (double)").asDouble();

    double Tact = rf.check("Tact",
                     yarp::os::Value(20),
                     "Tact (double)").asDouble();

    double Tinact = rf.check("Tinact",
                     yarp::os::Value(10),
                     "Tinact (double)").asDouble();

    double Tfree = rf.check("Tfree",
                            yarp::os::Value(5),
                            "Tfree (double)").asDouble();

    double Tevent = rf.check("Tevent",
                             yarp::os::Value(2),
                             "Tevent (double)").asDouble();

    double SigX = rf.check("SigX",
                           yarp::os::Value(5),
                           "SigX (double)").asDouble();

    double SigY = rf.check("SigY",
                           yarp::os::Value(5),
                           "SigY (double)").asDouble();

    double SigXY = rf.check("SigXY",
                            yarp::os::Value(0),
                            "SigXY (double)").asDouble();

    bool Fixedshape = rf.check("Fixedshape",
                               yarp::os::Value(false),
                               "Fixedshape (double)").asBool();

    int Regrate = rf.check("Regrate",
                              yarp::os::Value(50),
                              "Regrate (double)").asInt();

    double Maxdist = rf.check("Maxdist",
                              yarp::os::Value(10),
                              "Maxdist (double)").asDouble();

    double decay_tau = rf.check("decay_tau",
                         yarp::os::Value(10000),
                         "decay rate (double)").asDouble();



    
    eventBottleManager.setAllParameters(alphaShape, alphaPos, Tact, Tinact,
                                        Tfree, Tevent, SigX, SigY, SigXY,
                                        Fixedshape, Regrate, Maxdist,
                                        decay_tau);

   /* now open the manager to do the work */
    if(!eventBottleManager.open(moduleName))
    {
        std::cerr << " : Unable to open ports" << std::endl;
        return false;
    }

    closing = false;
    return true ;

}

/******************************************************************************/
bool EventClustering::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    return true;
}

/******************************************************************************/
bool EventClustering::close()
{
    closing = true;
    rpcPort.close();
    eventBottleManager.close();

    return true;
}

/******************************************************************************/
bool EventClustering::updateModule()
{
    return !closing;
}

/******************************************************************************/
double EventClustering::getPeriod()
{
    return 0.1;
}

/******************************************************************************/
//EventBottleManager
/******************************************************************************/

void EventBottleManager::setAllParameters(double alpha_shape, double alpha_pos,
                                     double Tact, double Tinact, double Tfree,
                                     double Tevent, double SigX, double SigY,
                                     double SigXY, bool Fixedshape, int Regrate,
                                     double Maxdist, double decay_tau)
{
    tracker_pool_left.setComparisonParams(Maxdist);
    tracker_pool_left.setDecayParams(decay_tau, Tact, Tinact, Tfree, Tevent,
                                     Regrate);
    tracker_pool_left.setInitialParams(SigX, SigY, SigXY, alpha_pos,
                                       alpha_shape, Fixedshape);

    tracker_pool_right.setComparisonParams(Maxdist);
    tracker_pool_right.setDecayParams(decay_tau, Tact, Tinact, Tfree, Tevent,
                                     Regrate);
    tracker_pool_right.setInitialParams(SigX, SigY, SigXY, alpha_pos,
                                       alpha_shape, Fixedshape);
}

/******************************************************************************/
bool EventBottleManager::open(std::string moduleName)
{
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool success1 = yarp::os::BufferedPort< emorph::vBottle >::open(inPortName);


    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool success2 = outPort.open(outPortName);

    if(!success1 || !success2) {
        yarp::os::BufferedPort< emorph::vBottle >::close();
        outPort.close();
    }

    return success1 && success2;
}

/******************************************************************************/
void EventBottleManager::close()
{
    outPort.close();
    BufferedPort<emorph::vBottle >::close();
}

/******************************************************************************/
void EventBottleManager::interrupt()
{
    outPort.interrupt();
    BufferedPort< emorph::vBottle >::interrupt();
}

/******************************************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{

    // prepare output vBottle with address events extended with
    // cluster ID (aec) and cluster events (clep)
    emorph::vBottle &evtCluster = outPort.prepare();
    evtCluster.clear();
    std::vector<emorph::ClusterEventGauss> clEvts;
    std::vector<emorph::ClusterEventGauss>::iterator ceit;


    //create event queue and iterator
    emorph::vQueue q;
    emorph::vQueue::iterator qi;
    bot.getAll(q);

    // checks for empty or non valid queue????
    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        unsigned long ev_t      = aep->getStamp();
        short ev_x              = aep->getX();
        short ev_y              = aep->getY();
        short pol               = aep->getPolarity();
        short channel           = aep->getChannel();

        //not sure why this check is really needed?
        if(!(ev_x>=0 && ev_x <= 127 && ev_y>=0 && ev_y <= 127)) continue;

        if (channel == 0) //Process events for left camera
        {
            clEvts.clear();
            int clusterAssignedTo = tracker_pool_left.update(*aep, clEvts);

            //add the event depending if it was assigned a cluster
            if(clusterAssignedTo >= 0) {
                emorph::AddressEventClustered aec = *aep;
                aec.setID(clusterAssignedTo);
                evtCluster.addEvent(aec);
            } else {
                evtCluster.addEvent(*aep);
            }

            //add the clusterEvents
            for(ceit = clEvts.begin(); ceit != clEvts.end(); ceit++) {
                ceit->setChannel(0);
                evtCluster.addEvent(*ceit);
            }

        }
        else
        {
            clEvts.clear();
            int clusterAssignedTo = tracker_pool_right.update(*aep, clEvts);

            //add the event depending if it was assigned a cluster
            if(clusterAssignedTo >= 0) {
                emorph::AddressEventClustered aec = *aep;
                aec.setID(clusterAssignedTo);
                evtCluster.addEvent(aec);
            } else {
                evtCluster.addEvent(*aep);
            }

            //add the clusterEvents
            for(ceit = clEvts.begin(); ceit != clEvts.end(); ceit++) {
                ceit->setChannel(1);
                evtCluster.addEvent(*ceit);

            }

        }
    }


    outPort.write();
}

//empty line to make gcc happy
