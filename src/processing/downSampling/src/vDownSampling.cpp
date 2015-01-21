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

#include "vDownSampling.h"


using namespace emorph;

/**********************************************************/
bool vDownSamplingModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name", yarp::os::Value("vDownSampling"),
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
    
    double samplingFactor = rf.check("sampleBy", yarp::os::Value(2.0), "asdasd").asDouble();

    /* create the thread and pass pointers to the module parameters */
    eventBottleManager = new EventBottleManager;
    eventBottleManager->open(moduleName, samplingFactor);


    return true ;
}

/**********************************************************/
bool vDownSamplingModule::interruptModule()
{
    rpcPort.interrupt();
    eventBottleManager->interrupt();
    return true;
}

/**********************************************************/
bool vDownSamplingModule::close()
{
    rpcPort.close();
    eventBottleManager->close();
    delete eventBottleManager;
    return true;
}

/**********************************************************/
bool vDownSamplingModule::updateModule()
{
    return true;
}

/**********************************************************/
double vDownSamplingModule::getPeriod()
{
    return 0.1;
}

bool vDownSamplingModule::respond(const yarp::os::Bottle &command,
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
bool EventBottleManager::open(const std::string &name, double samplingFactor)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);

    std::fprintf(stdout, "Opened ports. Starting downSamplingProcessor \n");

    /* create downSamplingProcess instance */
    downSamplingInstance = new downSamplingProcessor;

    downSamplingInstance->setSamplingFactor(samplingFactor);
    downSamplingInstance->addWeights(1/(2*downSamplingInstance->getSamplingFactor()));

    std::fprintf(stdout, "created downSamplingProcessor with samplingFactor %f and weights %f \n", downSamplingInstance->getSamplingFactor(), 1/(2*downSamplingInstance->getSamplingFactor()));

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

    short ev_x;
    short ev_y;
    short pol;
    short channel;

    unsigned long ev_t;

//    std::fprintf(stdout, "  [eventBottleManager OnREAD]:: Got events.\n");

    for(qi = q.begin(); qi != q.end(); qi++)
    {
        int addedEvents =0;
        // get events
        ev_t = (*qi)->getStamp();

        AddressEvent *aep = (*qi)->getAs<AddressEvent>(); // address event from the input vBottle

        //process
        ev_x    = aep->getX();
        ev_y    = aep->getY();
        pol     = aep->getPolarity(); pol = pol*2 - 1;
        channel = aep->getChannel();

        if (channel == 0)
        {
//            std::fprintf(stdout, "Event x, y: %d %d %d %d %d\n", ev_x, ev_y, int(ev_x/2), int(ev_y/2), pol);

            int dwnSampleEvent = downSamplingInstance->downSampling(ev_x, ev_y, pol);
            if(dwnSampleEvent == 1 || dwnSampleEvent == -1)
            {
                AddressEvent ae(*aep);


                ae.setX( int(ev_x/downSamplingInstance->getSamplingFactor())  + int(double(128.0/2.0) - 128.0/(2.0*downSamplingInstance->getSamplingFactor())) );
                ae.setY( int(ev_y/downSamplingInstance->getSamplingFactor())  + int(double(128.0/2.0) - 128.0/(2.0*downSamplingInstance->getSamplingFactor())) );
                ae.setPolarity(pol);
                ae.setChannel(channel);
                //ae->encode();

                outBottle.addEvent(ae);
                addedEvents++;
            }
        }
    }
    //send on the processed events
    outPort.write();

    //outBottle.clear();

}

//empty line to make gcc happy
