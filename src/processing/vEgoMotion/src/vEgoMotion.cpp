/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
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

#include "vEgoMotion.h"

/**********************************************************/
bool vEgoMotionModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name", yarp::os::Value("vEgoMotion"),
                                      "module name (string)").asString();
    setName(moduleName.c_str());

    //open and attach the rpc port
    std::string rpcPortName  =  "/" + moduleName + "rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << getName() << " : Unable to open rpc port at " <<
                     rpcPortName << std::endl;
        return false;
    }

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcPort);


    //set other variables we need from the
    std::string fileName = rf.check("variable",
                        yarp::os::Value("variable_defualt"),
                        "variable description").asString();
    
    /* create the thread and pass pointers to the module parameters */
    eventBottleManager = new EventBottleManager;
    eventBottleManager->open(moduleName);

    return true ;
}

/**********************************************************/
bool vEgoMotionModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager->interrupt();
    return true;
}

/**********************************************************/
bool vEgoMotionModule::close()
{
    rpcPort.close();
    eventBottleManager->close();
    delete eventBottleManager;
    return true;
}

/**********************************************************/
bool vEgoMotionModule::updateModule()
{
    return true;
}

/**********************************************************/
double vEgoMotionModule::getPeriod()
{
    return 0.1;
}

bool vEgoMotionModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    
}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);
    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    this->close();
    outPort.close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    this->interrupt();
    outPort.interrupt();

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q;
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    // get the event queue in the vBottle bot
    bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::OpticalFlowEvent *ofp = (*qi)->getAs<emorph::OpticalFlowEvent>();

        x = ofp->getX();
        y = ofp->getY();
        vx = ofp->getVx();
        vy = ofp->getVy();
        ts = ofp->getStamp();

        //process

        //add events that need to be added to the out bottle
        outBottle.addEvent(**qi);

    }
    //send on the processed events
    outPort.write();

}

/**********************************************************/
/*void EventBottleManager::computeStat(double _vx, double _vy, uint _dim, yarp::sig::Vector &_mean, yarp::sig::Matrix &_covariance)
{
    double m_vx, m_vy;
    double sigma_vx, sigma_vy, sigma_vxy;

    m_vx = 0;
    m_vy = 0;
    sigma_vx = 0;
    sigma_vy = 0;
    sigma_vxy = 0;

    //compute mean
    for(int i=0; i<_dim; i++)
    {
        m_vx = m_vx + _vx[i];
        m_vy = m_vy + _vy[i];
    }

    //compute variance and covariance
    for(int i=0; i<_dim; i++)
    {
        sigma_vx = sigma_vx + (_vx[i] - m_vx)*(_vx[i] - m_vx);
        sigma_vy = sigma_vy + (_vy[i] - m_vy)*(_vy[i] - m_vy);
        sigma_vxy = sigma_vxy + (_vx[i] - m_vx)*(_vy[i] - m_vy);
    }

    m_vx = m_vx/_dim;
    m_vy = m_vy/_dim;

    sigma_vx = sigma_vx/_dim;
    sigma_vy = sigma_vy/_dim;
    sigma_vxy = sigma_vxy/_dim;

    //create the mean vector
    _mean(0) = m_vx;
    _mean(1) = m_vy;

    //create the covariance matrix
    _covariance(0,0) = sigma_vx;
    _covariance(0,1) = sigma_vxy;
    _covariance(1,0) = sigma_vxy;
    _covariance(1,1) = sigma_vy;

}
*/
//empty line to make gcc happy
