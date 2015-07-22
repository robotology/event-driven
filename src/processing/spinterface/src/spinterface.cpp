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

#include "spinterface.h"

/**********************************************************/
bool vSpinInterface::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vTemplate")).asString();
    setName(moduleName.c_str());


    //set other variables we need from the
    std::string fileName = rf.check("variable",
                        yarp::os::Value("variable_defualt"),
                        "variable description").asString();
    
    /* create the thread and pass pointers to the module parameters */
    return ioManager.open(moduleName);

}

/**********************************************************/
bool vSpinInterface::interruptModule()
{
    ioManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vSpinInterface::close()
{
    ioManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vSpinInterface::updateModule()
{
    return true;
}

/**********************************************************/
double vSpinInterface::getPeriod()
{
    return 1;
}

/**********************************************************/
YARPspinIO::YARPspinIO()
{

    //here we should initialise the module
    
}
/**********************************************************/
bool YARPspinIO::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool state1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    bool state2 = outPort.open(outPortName);
    return state1 && state2;
}

/**********************************************************/
void YARPspinIO::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void YARPspinIO::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void YARPspinIO::onRead(emorph::vBottle &bot)
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

        //perhaps you need to filter for a certain type of event?
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;

        //process

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);


    }
    //send on the processed events
    outPort.write();

}

//empty line to make gcc happy
