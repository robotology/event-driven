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
                                      yarp::os::Value("vCluster")
                                      ).asString();

    setName(moduleName.c_str());

    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << rpcPortName << " : Unable to open port" << std::endl;
        return false;
    }
    attach(rpcPort);


    //how quickly cluster shape changes given new information
    double alphaShape =
            rf.check("alphaShape", yarp::os::Value(0.01)).asDouble();
    //how quickly cluster position changes given new information
    double alphaPos = rf.check("alphaPos", yarp::os::Value(0.1)).asDouble();
    //at what threshold a cluster becomes active given activity
    double Tact = rf.check("tAct", yarp::os::Value(20)).asDouble();
    //at what threshold an active cluster becomes inactive given no activity
    double Tinact = rf.check("tInact", yarp::os::Value(10)).asDouble();
    //at what threshold an inactive cluster becomes free
    double Tfree = rf.check("tFree", yarp::os::Value(5)).asDouble();
    //maximum period at which a cluster can output a cluster event
    double Tevent = rf.check("tClusRefr", yarp::os::Value(2)).asDouble();
    //cluster initialisation parameters (size in x, y and angle between x&y
    double SigX = rf.check("sigX", yarp::os::Value(5)).asDouble();
    double SigY = rf.check("sigY", yarp::os::Value(5)).asDouble();
    double SigXY = rf.check("sigXY", yarp::os::Value(0)).asDouble();
    //are clusters fixed to circular gaussians?
    bool Fixedshape = rf.check("fixedShape", yarp::os::Value(false)).asBool();
    //how often clusters are decayed (computation trade-off)
    int Regrate = rf.check("regRate", yarp::os::Value(50)).asInt();
    //maximum distance an event can be from the centre of the cluster
    double Maxdist = rf.check("maxDist", yarp::os::Value(10)).asDouble();
    //how slowly events decay
    double decay_tau = rf.check("decay", yarp::os::Value(10000)).asDouble();
    //is there a limit on the number of clusters?
    double clusterLimit =
            rf.check("clusterLimit", yarp::os::Value(-1)).asDouble();




    eventBottleManager.setAllParameters(alphaShape, alphaPos, Tact, Tinact,
                                        Tfree, Tevent, SigX, SigY, SigXY,
                                        Fixedshape, Regrate, Maxdist,
                                        decay_tau, clusterLimit);

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
                                          double Tact, double Tinact,
                                          double Tfree,
                                          double Tevent, double SigX,
                                          double SigY,
                                          double SigXY, bool Fixedshape,
                                          int Regrate,
                                          double Maxdist, double decay_tau,
                                          double clusterLimit)
{
    //left
    tracker_pool_left.setComparisonParams(Maxdist);
    tracker_pool_left.setDecayParams(decay_tau, Tact, Tinact, Tfree, Tevent,
                                     Regrate);
    tracker_pool_left.setInitialParams(SigX, SigY, SigXY, alpha_pos,
                                       alpha_shape, Fixedshape);
    tracker_pool_left.setClusterLimit(clusterLimit);

    //right
    tracker_pool_right.setComparisonParams(Maxdist);
    tracker_pool_right.setDecayParams(decay_tau, Tact, Tinact, Tfree, Tevent,
                                     Regrate);
    tracker_pool_right.setInitialParams(SigX, SigY, SigXY, alpha_pos,
                                       alpha_shape, Fixedshape);
    tracker_pool_right.setClusterLimit(clusterLimit);

}

/******************************************************************************/
bool EventBottleManager::open(std::string moduleName)
{
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool success1 = yarp::os::BufferedPort< ev::vBottle >::open(inPortName);


    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool success2 = outPort.open(outPortName);

    if(!success1 || !success2) {
        yarp::os::BufferedPort< ev::vBottle >::close();
        outPort.close();
    }

    return success1 && success2;
}

/******************************************************************************/
void EventBottleManager::close()
{
    outPort.close();
    BufferedPort<ev::vBottle >::close();
}

/******************************************************************************/
void EventBottleManager::interrupt()
{
    outPort.interrupt();
    BufferedPort< ev::vBottle >::interrupt();
}

/******************************************************************************/
void EventBottleManager::onRead(ev::vBottle &bot)
{

    // prepare output vBottle with address events extended with
    // cluster ID (aec) and cluster events (clep)
    ev::vBottle &evtCluster = outPort.prepare();
    evtCluster.clear();
    std::vector<ev::event<ev::GaussianAE> > clEvts;
    std::vector<ev::event<ev::GaussianAE> >::iterator ceit;


    //create event queue and iterator
    ev::vQueue q = bot.getAll();
    ev::vQueue::iterator qi;
    //bot.getAll(q);

    // checks for empty or non valid queue????
    for(qi = q.begin(); qi != q.end(); qi++)
    {
        auto aep = ev::as_event<ev::AddressEvent>(*qi);
        if(!aep) continue;

        short channel           = aep->getChannel();

        if (channel == 0) //Process events for left camera
        {
            clEvts.clear();
            int clusterAssignedTo = tracker_pool_left.update(aep, clEvts);

            //add the event depending if it was assigned a cluster
            if(clusterAssignedTo >= 0) {
                auto aec = ev::make_event<ev::LabelledAE>(aep);
                aec->ID = clusterAssignedTo;
                evtCluster.addEvent(aec);
            } else {
                evtCluster.addEvent(aep);
            }

            //add the clusterEvents
            for(ceit = clEvts.begin(); ceit != clEvts.end(); ceit++) {
                (*ceit)->setChannel(0);
                evtCluster.addEvent(*ceit);
            }

        }
        else
        {
            clEvts.clear();
            int clusterAssignedTo = tracker_pool_right.update(aep, clEvts);

            //add the event depending if it was assigned a cluster
            if(clusterAssignedTo >= 0) {
                auto aec = ev::make_event<ev::LabelledAE>(aep);
                aec->ID = clusterAssignedTo;
                evtCluster.addEvent(aec);
            } else {
                evtCluster.addEvent(aep);
            }

            //add the clusterEvents
            for(ceit = clEvts.begin(); ceit != clEvts.end(); ceit++) {
                (*ceit)->setChannel(1);
                evtCluster.addEvent(*ceit);

            }

        }
    }


    outPort.write();
}

//empty line to make gcc happy
