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

#include "spinconverter.h"

/**********************************************************/
bool spinconverterModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("spinconverter")).asString();
    setName(moduleName.c_str());

    /* create the thread and pass pointers to the module parameters */
    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool spinconverterModule::interruptModule()
{
    eventBottleManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool spinconverterModule::close()
{
    eventBottleManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool spinconverterModule::updateModule()
{
    return true;
}

/**********************************************************/
double spinconverterModule::getPeriod()
{
    return 0.1;
}

bool spinconverterModule::respond(const yarp::os::Bottle &command,
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
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void EventBottleManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void EventBottleManager::onRead(emorph::vBottle &bot)
{
    //create event queue
    emorph::vQueue q = bot.getAll();
    //create queue iterator
    emorph::vQueue::iterator qi;
    
    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle = bot; //start with the incoming bottle

    // get the event queue in the vBottle bot
    //bot.getAll(q);

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        //unwrap timestamp
        unsigned long int ts = unwrapper((*qi)->getStamp());

        emorph::NeuronIDEvent *ne = (*qi)->getAs<emorph::NeuronIDEvent>();
        if(!ne) continue;

        std::cout << ne->getID() << std::endl;

        //process


        //add events that need to be added to the out bottle
//        emorph::ClusterEventGauss ceg;
//        outBottle.addEvent(ceg);


    }
    //send on the processed events
    outPort.write();

}

//empty line to make gcc happy
