/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#include "autoSaccade.h"

/**********************************************************/
bool saccadeModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("autoSaccade")).asString();
    setName(moduleName.c_str());

    //open and attach the rpc port
    std::string rpcPortName  =  "/" + moduleName + "/rpc:i";

    if (!rpcPort.open(rpcPortName))
    {
        std::cerr << getName() << " : Unable to open rpc port at " <<
                     rpcPortName << std::endl;
        return false;
    }

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcPort);


    //set other variables we need from the
    checkPeriod = rf.check("checkPeriod",
                           yarp::os::Value(0.8)).asDouble();
    minVpS = rf.check("minVpS", yarp::os::Value(50)).asDouble();
    prevStamp = 4294967295; //max value


    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool saccadeModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    return true;
}

/**********************************************************/
bool saccadeModule::close()
{
    rpcPort.close();
    eventBottleManager.close();
    return true;
}

/**********************************************************/
bool saccadeModule::updateModule()
{
    //if there is no connection don't do anything yet
    if(!eventBottleManager.getInputCount()) return true;

    //check the last time stamp and count
    unsigned long int latestStamp = eventBottleManager.getTime();
    unsigned long int vCount = 100000 * eventBottleManager.popCount();

    double vPeriod = latestStamp - prevStamp;
    prevStamp = latestStamp;

    //this should only occur on first bottle to initialise
    if(vPeriod < 0) return true;

    if(vPeriod == 0 || (vCount / vPeriod) < minVpS) {
        //perform saccade
        std::cout << "perform saccade" << std::endl;
    } else {
        std::cout << vPeriod / 100000 << std::endl;
    }

    return true;
}

/**********************************************************/
double saccadeModule::getPeriod()
{
    return checkPeriod;
}

bool saccadeModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    vCount = 0;
    latestStamp = 0;

    
}
/**********************************************************/
bool EventBottleManager::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    //close ports
    this->close();

    //remember to also deallocate any memory allocated by this class
}

/**********************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    this->interrupt();

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q;
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // get the event queue in the vBottle bot
    bot.getSorted<emorph::AddressEvent>(q);

    latestStamp = unwrapper(q.back()->getStamp());

    mutex.wait();
    vCount += q.size();
    mutex.post();


}

unsigned long int EventBottleManager::getTime()
{
    return latestStamp;

}

unsigned long int EventBottleManager::popCount()
{
    mutex.wait();
    unsigned long int r = vCount;
    vCount = 0;
    mutex.post();
    return r;

}

//empty line to make gcc happy
