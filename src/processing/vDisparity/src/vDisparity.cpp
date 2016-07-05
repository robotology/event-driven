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
    //std::string moduleName =
    //        rf.check("name", yarp::os::Value("vDisparity")).asString();
    setName(rf.check("name", yarp::os::Value("vDisparity")).asString().c_str());

    //set other variables we need
    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* create the thread and pass pointers to the module parameters */
    disparityManager = new vDisparityManager(rf.check("width", yarp::os::Value(128)).asInt(),
                                             rf.check("height", yarp::os::Value(128)).asInt(),
                                             rf.check("nEvents", yarp::os::Value(200)).asInt(),
                                             rf.check("ori", yarp::os::Value(1)).asInt(),
                                             rf.check("phases", yarp::os::Value(5)).asInt(),
                                             rf.check("disparity", yarp::os::Value(14)).asInt(),
                                             rf.check("stdsperlambda", yarp::os::Value(6.0)).asDouble());

    return disparityManager->open(getName(), strict);
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
vDisparityManager::vDisparityManager(int width, int height, int nEvents, int numberOri, int numberPhases, int maxDisparity, double stdsPerLambda)
{
    this->width = width;
    this->height = height;
    this->winsize = maxDisparity * 8.0 / 3.0;

    //enforce an odd number of phases
    numberPhases = 2 * (numberPhases / 2) + 1;
    double sigma = maxDisparity * 8.0 / (stdsPerLambda * 3.0);

    //create the filterbank
    std::cout << "Initialising Filterbank" << std::endl;
    filters.resize(numberPhases);
    std::cout << "Phases:";
    for(int i = 0; i < numberPhases; i++) {
        filters[i].setCenter(width/2, height/2);
        int lambda;
        if(numberPhases == 1)
            lambda = 0;
        else
            lambda = -maxDisparity + i * maxDisparity * 2.0 / (numberPhases - 1) + 0.5;
        filters[i].setParameters(sigma, stdsPerLambda, M_PI * 0.5, lambda);
        std::cout << " " << lambda;
    }

    std::cout << std::endl;


    filterweights.resize(numberPhases);
    //create the filter weights
    //this needs to be more robust (i.e. use filter disparity to set weight)
    for(unsigned int i = 0; i < filters.size(); i++) {

        if(i < (filters.size() - 1) / 2)
        {
            filterweights[i] = -1;
        }
        else
        {
            if(i == (filters.size() - 1) / 2)
                filterweights[i] = 0;
            else
                filterweights[i] = 1;
        }
    }

    //create the surface representations
    fifoLeft = new emorph::fixedSurface(nEvents, width, height);
    fifoRight = new emorph::fixedSurface(nEvents, width, height);

	gazecontrol = 0;
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

    if(!yarp::os::BufferedPort<emorph::vBottle>::open("/" + name + "/vBottle:i"))
        return false;

    if(!outPort.open("/" + name + "/vBottle:o"))
        return false;

    if(!scopeOut.open("/" + name + "/scope:o"))
        return false;

    if(!debugOut.open("/" + name + "/debug:o"))
        return false;

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + name);
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else
        std::cerr << "Gaze Driver not opened and will not be used" << std::endl;


    return true;
}

/**********************************************************/
void vDisparityManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
    scopeOut.close();
    debugOut.close();

    //close controller
    if(gazedriver.isValid())
    {
        gazecontrol->stopControl();
        gazedriver.close();
    }

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
    scopeOut.interrupt();
    debugOut.interrupt();

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
        if(abs(aep->getX() - width/2) > winsize / 2 || abs(aep->getY() - height/2) > winsize / 2) continue;

        if(aep->getChannel())
            fifoCurr = fifoRight;
        else
            fifoCurr = fifoLeft;

        //add event to the fifo
        emorph::vQueue removed = fifoCurr->addEvent(*aep);

        for(unsigned int i = 0; i < filters.size(); i++) {
            filters[i].process(*aep);
            filters[i].process(removed, -1.0);
        }

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);

    }

//    yarp::sig::Vector angles(3);
//    if(gazedriver.isValid())
//    {
//        gazecontrol->getAngles(angles);
//        std::cout << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;
//    }

    yarp::os::Bottle &scopebot = scopeOut.prepare();
    scopebot.clear();
    double respsum = 0.0;

    for(unsigned int i = 0; i < filters.size(); i++) {

        respsum += filterweights[i] * filters[i].getResponse();
        if(filters[i].getResponse() > 0)
            scopebot.addDouble(filters[i].getResponse());
        else
            scopebot.addDouble(0.0);
    }
    scopebot.addDouble(respsum);
    scopeOut.write();

    static int i = 0;
    if(i % 10 == 0 && debugOut.getOutputCount()) {

        yarp::sig::ImageOf< yarp::sig::PixelBgr > &image = debugOut.prepare();
        image.resize(height, width);
        image.zero();

        emorph::vQueue curwin = fifoLeft->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            emorph::AddressEvent *v = curwin[j]->getAs<emorph::AddressEvent>();
            if(!v) continue;

            image(v->getY(), width - 1 - v->getX()) = yarp::sig::PixelBgr(255, 255, 0);
        }

        curwin = fifoRight->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            emorph::AddressEvent *v = curwin[j]->getAs<emorph::AddressEvent>();
            if(!v) continue;

            image(v->getY(), width - 1 - v->getX()) = yarp::sig::PixelBgr(255, 0, 255);
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
