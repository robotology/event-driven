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
    double sigma = rf.check("sigma", yarp::os::Value(5)).asDouble();
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
    this->numberPhases = 2 * (numberPhases / 2) + 1;
    this->sigma = sigma;
    this->winsize = winsize;

    //fifoLeft = new emorph::temporalSurface(width, height, tempWin * 7812.5);
    //fifoRight = new emorph::temporalSurface(width, height, tempWin * 7812.5);

    fifoLeft = new emorph::fixedSurface(200, width, height);
    fifoRight = new emorph::fixedSurface(200, width, height);

    std::cout << "Initialising Filterbank" << std::endl;
    filters.resize(this->numberPhases);
    std::cout << "Phases:";
    for(int i = 0; i < this->numberPhases; i++) {
        filters[i].setCenter(64, 64);
        int lambda = -winsize + i * (double)winsize / (this->numberPhases / 2) + 0.5;
        filters[i].setParameters(sigma, M_PI * 0.5, lambda);
        std::cout << " " << lambda;
    }
    std::cout << std::endl;

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

    if(!debugOut.open("/"+name+"/debug:o"))
            return false;

    return check1 && check2 && check3;
}

/**********************************************************/
void vDisparityManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class
    delete fifoLeft;
    delete fifoRight;

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
    this->getEnvelope(st);
    outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {

        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        //consider only events around the center
        if(abs(aep->getX() - 64) > winsize / 2 || abs(aep->getY() - 64) > winsize / 2) continue;

        if(aep->getChannel())
            fifoCurr = fifoRight;
        else
            fifoCurr = fifoLeft;

        //add event to the fifo
        emorph::vQueue removed = fifoCurr->addEvent(*aep);

//        if(!removed.size()) continue;

//        emorph::vQueue::iterator re = removed.end() - 1;
//        emorph::AddressEvent *rep = (*re)->getAs<emorph::AddressEvent>();

        //process
//        response = filters.process(fifo->getSurf()) + filters.process(*rep);
        for(unsigned int i = 0; i < filters.size(); i++) {
            filters[i].process(*aep);
            filters[i].process(removed, -1.0);
        }

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);

    }

    yarp::os::Bottle &scopebot = scopeOut.prepare();
    scopebot.clear();
    for(unsigned int i = 0; i < filters.size(); i++) {
        if(filters[i].getResponse() > 0)
            scopebot.addDouble(filters[i].getResponse());
        else
            scopebot.addDouble(0.0);
    }
    scopeOut.write();

    static int i = 0;
    if(i % 10 == 0 && debugOut.getOutputCount()) {

        yarp::sig::ImageOf< yarp::sig::PixelBgr > &image = debugOut.prepare();
        image.resize(128, 128);
        image.zero();

        emorph::vQueue curwin = fifoLeft->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            emorph::AddressEvent *v = curwin[j]->getAs<emorph::AddressEvent>();
            if(!v) continue;

            image(v->getY(), 127 - v->getX()) = yarp::sig::PixelBgr(255, 255, 0);
        }

        curwin = fifoRight->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            emorph::AddressEvent *v = curwin[j]->getAs<emorph::AddressEvent>();
            if(!v) continue;

            image(v->getY(), 127 - v->getX()) = yarp::sig::PixelBgr(255, 0, 255);
        }

        debugOut.write();
    }

    //send on the processed events
//    if(strictness)
//        outPort.writeStrict();
//    else
//        outPort.write();

}

//empty line to make gcc happy
