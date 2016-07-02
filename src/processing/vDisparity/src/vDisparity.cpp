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

#include "vDisparity.h"

/**********************************************************/
bool vDisparityModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vDisparity")).asString();
    setName(moduleName.c_str());

    //set other variables we need
    bool strict = rf.check("strict", yarp::os::Value("true")).asBool();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int tempWin = rf.check("tempWin", yarp::os::Value(50)).asInt();
    int numberOri = rf.check("ori", yarp::os::Value(1)).asInt();
    int numberPhases = rf.check("phases", yarp::os::Value(1)).asInt();
    double sigma = rf.check("sigma", yarp::os::Value(16)).asDouble();
    int winsize = rf.check("winsize", yarp::os::Value(32)).asInt();
    
    /* create the thread and pass pointers to the module parameters */
    disparityManager = new vDisparityManager(width, height, tempWin, numberOri, numberPhases, sigma, winsize);

    return disparityManager->open(moduleName, strict);
}

/**********************************************************/
bool vDisparityModule::interruptModule()
{
    disparityManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vDisparityModule::close()
{
    disparityManager->close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vDisparityModule::updateModule()
{
    return true;
}

/**********************************************************/
double vDisparityModule::getPeriod()
{
    return 0.1;
}

bool vDisparityModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    //fill in all command/response plus module update methods here
    return true;
}


/**********************************************************/
vDisparityManager::vDisparityManager(int width, int height, int tempWin, int numberOri, int numberPhases, double sigma, int winsize)
{
    this->width = width;
    this->height = height;
    this->tempWin = tempWin;
    this->numberOri = numberOri;
    this->numberPhases = numberPhases;
    this->sigma = sigma;
    this->winsize = winsize;

    fifo = new emorph::temporalSurface(width, height, tempWin * 7812.5);

    //set filter parameters
    filters.setCenter(64, 64);
    filters.setParameters(sigma, 0, 0);
    
}
/**********************************************************/
bool vDisparityManager::open(const std::string &name, bool strictness)
{
    //and open the input port
    if(strictness) {
        this->strictness = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    bool check1 = yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    std::string scopePortName = "/" + name + "/scope:o";
    bool check3 = scopeOut.open(scopePortName);

    return check1 && check2 && check3;
}

/**********************************************************/
void vDisparityManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
    delete fifo;

}

/**********************************************************/
void vDisparityManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void vDisparityManager::onRead(emorph::vBottle &bot)
{
    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    emorph::vQueue removed;

    double response = 0;

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        //consider only events around the center
        if(abs(aep->getX() - 64) > winsize / 2 || abs(aep->getY() - 64) > winsize / 2) continue;

        //add event to the fifo
        removed = fifo->addEvent(*aep);

//        if(!removed.size()) continue;

//        emorph::vQueue::iterator re = removed.end() - 1;
//        emorph::AddressEvent *rep = (*re)->getAs<emorph::AddressEvent>();

        //process
//        response = filters.process(*aep) + filters.process(*rep);
        response = filters.process(fifo->getSurf()) + filters.process(removed);

        std::cout << response << std::endl;

        yarp::os::Bottle &scopebot = scopeOut.prepare();
        scopebot.clear();
        scopebot.addDouble(response);
        scopeOut.write();

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);

    }
    //send on the processed events
//    outPort.write();

}

//empty line to make gcc happy
