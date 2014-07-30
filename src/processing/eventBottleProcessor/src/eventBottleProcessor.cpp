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

#include "eventBottleProcessor.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace emorph::ecodec;

/**********************************************************/
bool EventBottleProcessor::configure(yarp::os::ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("eventBottleProcessor"), "module name (string)").asString();

    setName(moduleName.c_str());

    rpcPortName  =  "/";
    rpcPortName +=  getName();
    rpcPortName +=  "/rpc:i";

    if (!rpcPort.open(rpcPortName.c_str()))
    {
        fprintf(stdout, "%s : Unable to open port %s\n", getName().c_str(), rpcPortName.c_str());
        return false;
    }

    attach(rpcPort);

    /* create the thread and pass pointers to the module parameters */
    eventBottleManager = new EventBottleManager( moduleName );

    /* now open the manager to do the work */
    eventBottleManager->open();

    return true ;
}

/**********************************************************/
bool EventBottleProcessor::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager->interrupt();
    return true;
}

/**********************************************************/
bool EventBottleProcessor::close()
{
    rpcPort.close();
    fprintf(stdout, "starting the shutdown procedure\n");

    eventBottleManager->close();
    fprintf(stdout, "deleting thread\n");
    delete eventBottleManager;
    fprintf(stdout, "done deleting thread\n");
    return true;
}

/**********************************************************/
bool EventBottleProcessor::updateModule()
{
    return !closing;
}

/**********************************************************/
double EventBottleProcessor::getPeriod()
{
    return 0.1;
}

/**********************************************************/
EventBottleManager::~EventBottleManager()
{

}

/**********************************************************/
EventBottleManager::EventBottleManager( const string &moduleName )
{
    fprintf(stdout,"initialising Variables\n");
    this->moduleName = moduleName;
}

/**********************************************************/
bool EventBottleManager::open()
{
    this->useCallback();

    //create all ports
    inPortName = "/" + moduleName + "/in:i";
    BufferedPort<eventBottle >::open( inPortName.c_str() );

    outPortName = "/" + moduleName + "/out:o";
    outPort.open( outPortName.c_str() );

    return true;
}

/**********************************************************/
void EventBottleManager::close()
{
    fprintf(stdout,"now closing ports...\n");
    outPort.close();
    BufferedPort<eventBottle >::close();
    fprintf(stdout,"finished closing the read port...\n");
}

/**********************************************************/
void EventBottleManager::interrupt()
{
    fprintf(stdout,"cleaning up...\n");
    fprintf(stdout,"attempting to interrupt ports\n");
    BufferedPort<eventBottle >::interrupt();
    fprintf(stdout,"finished interrupt ports\n");
}

/**********************************************************/
void EventBottleManager::onRead(eventBottle &bot)
{
    //create event queue
    eEventQueue q; 
    unsigned long ts;
    //decode packet
    Bottle out; 
    if(eEvent::decode(*bot.get_packet(),q)) 
    {   
        int size = q.size();
        Bottle &pack =  out.addList();  
        for (int e = 0; e < size; e++)
        {
            if(q[e] != 0)
            {
                if(q[e]->getType()=="TS") //identify the type of the packet (Time Stamp)
                {
                    TimeStamp* ptr=dynamic_cast<TimeStamp*>(q[e]);
                    ts = (unsigned long) ptr->getStamp();
                    //fprintf(stdout, "Size %d TimeStamp: %d", size, (unsigned int) ptr->getStamp());
                }
                else if (q[e]->getType()=="AE") //identify the type of the packet (Address Event)
                {
                    AddressEvent* ptr=dynamic_cast<AddressEvent*>(q[e]); //create the Address Event for the type
                    if(ptr->isValid()) 
                    {
                        short posX = ptr->getX();
                        short posY = ptr->getY();
                        short polarity = ptr->getPolarity();
                        short channel = ptr->getChannel();
                        
                        Bottle &tmp = pack.addList();                        
                        tmp.addDouble(ts);
                        tmp.addInt(posX);
                        tmp.addInt(posY);
                        tmp.addInt(polarity);
                        tmp.addInt(channel);
                        fprintf(stdout, "Size %d TimeStamp: %d Event: (X: %d  Y: %d  pol: %d  cha: %d) \n",size, ts, posX, posY, polarity, channel);
                    } 
                }
            }
        } 
        outPort.write(out);
    }   
    fprintf(stdout, "------------------------------------------------------------------\n");
}

//empty line to make gcc happy
