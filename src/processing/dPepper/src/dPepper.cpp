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

#include "dPepper.h"

/**********************************************************/
bool dPepperModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("dPepper")).asString();
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
    double temporalSize =
            rf.check("temporalSize", yarp::os::Value(10000)).asDouble();
    double spatialSize =
            rf.check("spatialSize", yarp::os::Value(1)).asDouble();

    eventBottleManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool dPepperModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager.interrupt();
    return true;
}

/**********************************************************/
bool dPepperModule::close()
{
    rpcPort.close();
    eventBottleManager.close();
    return true;
}

/**********************************************************/
bool dPepperModule::updateModule()
{
    return true;
}

/**********************************************************/
double dPepperModule::getPeriod()
{
    return 0.1;
}

bool dPepperModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
EventBottleManager::EventBottleManager()
{

    //here we should initialise the module
    temporalSize = 1000;
    spatialSize = 3;
    window.setWindowSize(temporalSize);
    
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

        window.addEvent(**qi);
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;
        //unwrap timestamp
        const emorph::vQueue &tw =
                window.getSpatialWindow(v->getX(), v->getY(), spatialSize);
        if(tw.size() > 1)
            outBottle.addEvent(**qi);

    }
    //send on the processed events
    outPort.write();

}

//empty line to make gcc happy
