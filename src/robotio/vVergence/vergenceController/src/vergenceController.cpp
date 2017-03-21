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

#include "vergenceController.h"

using namespace ev;

/**********************************************************/
bool vVergenceModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    setName(rf.check("name", yarp::os::Value("vVergence")).asString().c_str());

    //set other variables we need
    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    //open rpc port
    if(!rpcOut.open("/" + getName() + "/rpctrigger:i"))
        return false;

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcOut);

    /* create the thread and pass pointers to the module parameters */
    vergenceManager = new vVergenceManager(rf.check("width", yarp::os::Value(128)).asInt(),
                                             rf.check("height", yarp::os::Value(128)).asInt(),
                                             rf.check("nEvents", yarp::os::Value(200)).asInt(),
                                             rf.check("ori", yarp::os::Value(1)).asInt(),
                                             rf.check("phases", yarp::os::Value(7)).asInt(),
                                             rf.check("disparity", yarp::os::Value(14)).asInt(),
                                             rf.check("stdsperlambda", yarp::os::Value(6.0)).asDouble(),
                                             rf.check("threshold", yarp::os::Value(0.0)).asDouble());

    return vergenceManager->open(getName(), strict);
}

/**********************************************************/
bool vVergenceModule::interruptModule()
{
    rpcOut.interrupt();
    vergenceManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vVergenceModule::close()
{
    rpcOut.close();
    vergenceManager->close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vVergenceModule::updateModule()
{
    return true;
}

/**********************************************************/
double vVergenceModule::getPeriod()
{
    return 0.1;
}

bool vVergenceModule::respond(const yarp::os::Bottle &command,
                              yarp::os::Bottle &reply)
{
    if (command.get(0).asString() == "start") {
        reply.addString("Starting Verging...");
        this->vergenceManager->startVerging();
   }
    else if (command.get(0).asString() == "reset") {
        reply.addString("Resetting...");
        this->vergenceManager->resetVergence();

    }
    else if (command.get(0).asString() == "kp") {
        reply.addString("Setting kp...");
        this->vergenceManager->setkp(command.get(1).asDouble());

    }
    else if (command.get(0).asString() == "kd") {
        reply.addString("Setting kd...");
        this->vergenceManager->setkd(command.get(1).asDouble());

    }
    return true;
}


/**********************************************************/
vVergenceManager::vVergenceManager(int width, int height, int nEvents, int numberOri, int numberPhases, int maxDisparity, double stdsPerLambda, double threshold)
{
    this->width = width;
    this->height = height;
    this->winsize = maxDisparity * 8.0 / 3.0;
    this->numberOri = numberOri;
    this->numberPhases = numberPhases;
    this->threshold = threshold;
    this->kp = 5000;
    this->kd = 0;

    doVergence = true;

    //enforce an odd number of phases
    numberPhases = 2 * (numberPhases / 2) + 1;
    double sigma = maxDisparity * 8.0 / (stdsPerLambda * 3.0);

    //create the filterbank
    std::cout << "Initialising Filterbank..." << std::endl;

    filters.resize(numberOri * numberPhases);
    filterweights.resize(numberOri * numberPhases);

    totweights = 0;
    double minAngle = M_PI / 4;
    double maxAngle = 3 * M_PI / 4;
    double deltaAngle = (maxAngle - minAngle) / (numberOri - 1);
    for(int i = 0; i < numberOri; i++) {

        double scaledtheta;
        if(numberOri == 1) {
            scaledtheta = M_PI / 2;
        }
        else {
            scaledtheta = minAngle + i * deltaAngle;
        }

        std::cout << "Orientation " << scaledtheta * (180 / M_PI) << " with Phases (Weights): " << std::endl;
        for(int j = 0; j < numberPhases; j++) {
            filters[j + i * numberPhases].setCenter(width/2, height/2);
            int lambda;
            if(numberPhases == 1)
                lambda = 0;
            else
                lambda = -maxDisparity + j * maxDisparity * 2.0 / (numberPhases - 1) + 0.5;
            filters[j + i * numberPhases].setParameters(sigma, stdsPerLambda, scaledtheta, lambda);

             std::cout << lambda;

             //create the filter weights
//             if(j == (numberPhases - 1) / 2)
//                 filterweights[j + i * numberPhases] = 0;
//             else
//                 filterweights[j + i * numberPhases] = lambda / 2.0; // 4.0 / lambda;

             if(j < (numberPhases - 1) / 2)
                 filterweights[j + i * numberPhases] = -1;
             else if(j == (numberPhases - 1) / 2)
                 filterweights[j + i * numberPhases] = 0;
             else
                 filterweights[j + i * numberPhases] = 1;

             totweights += fabs(filterweights[j + i * numberPhases]);
             std::cout << " (" << filterweights[j + i * numberPhases] << ") ";

        }
        std::cout << std::endl << std::endl;
    }

    //create the surface representations
    fifoLeft = new ev::fixedSurface(nEvents, width, height);
    fifoRight = new ev::fixedSurface(nEvents, width, height);

    gazecontrol = 0;
    enccontrol = 0;
    poscontrol = 0;
    velcontrol = 0;
//    desiredvergence = 20;

    depth = 0;
    errorPrev = 0;
    error_d = 0;
}
/**********************************************************/
bool vVergenceManager::open(const std::string &name, bool strictness)
{
    //and open the input port
    if(strictness) {
        this->strictness = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();

    if(!yarp::os::BufferedPort<ev::vBottle>::open("/" + name + "/vBottle:i"))
        return false;

    if(!outPort.open("/" + name + "/vBottle:o"))
        return false;

    if(!scopeOut.open("/" + name + "/scope:o"))
        return false;

    if(!scopeFiltersOut.open("/" + name + "/scopefilters:o"))
        return false;

    if(!debugOut.open("/" + name + "/debug:o"))
        return false;

    yarp::os::Property optionsgaze;
    optionsgaze.put("device", "gazecontrollerclient");
    optionsgaze.put("local", "/" + name);
    optionsgaze.put("remote", "/iKinGazeCtrl");
    gazedriver.open(optionsgaze);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else
        std::cerr << "Gaze Driver not opened and will not be used" << std::endl;

    yarp::os::Property options;
    options.put("robot", "icub");
    options.put("device", "remote_controlboard");

    yarp::os::Value& robotname = options.find("robot");
    options.put("local", "/" + robotname.asString() + "/head/control");
    options.put("remote", "/" + robotname.asString() + "/head");
    encdriver.open(options);
    if(encdriver.isValid())
    {
        encdriver.view(enccontrol);
        encdriver.view(poscontrol);
        encdriver.view(velcontrol);
        encdriver.view(controlmode);

//        poscontrol->setRefSpeed(5, 10);
//        poscontrol->positionMove(5, desiredvergence);

        controlmode->setControlMode(5, VOCAB_CM_VELOCITY);
        int joints = 0;
        enccontrol->getAxes(&joints);
        encs.resize(joints);
    }
        else
    {
        std::cerr << "Encoder driver not opened" << std::endl;
    }


    return true;
}

/**********************************************************/
void vVergenceManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();
    scopeOut.close();
    scopeFiltersOut.close();
    debugOut.close();

    //close controller
    if(gazedriver.isValid())
    {
        gazecontrol->stopControl();
        gazedriver.close();
    }

    //close controller
    if(encdriver.isValid())
        encdriver.close();

    //remember to also deallocate any memory allocated by this class
    delete fifoLeft;
    delete fifoRight;

}

/**********************************************************/
void vVergenceManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
    scopeOut.interrupt();
    scopeFiltersOut.interrupt();
    debugOut.interrupt();

}

/**********************************************************/
void vVergenceManager::onRead(ev::vBottle &bot)
{
    /*prepare output vBottle with AEs extended with optical flow events*/
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st);
    outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();

    int countA = 0;
    int countR = 0;
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto aep = is_event<AE>(*qi);
        if(!aep) continue;

        //consider only events around the center
        if(abs(aep->x - width/2) > winsize / 2 || abs(aep->y - height/2) > winsize / 2) continue;

        if(aep->getChannel())
            fifoCurr = fifoRight;
        else
            fifoCurr = fifoLeft;

        //add event to the fifo
        ev::vQueue removed = fifoCurr->addEvent(aep);
        countA++;
        countR += removed.size();

        for(unsigned int i = 0; i < filters.size(); i++) {
            filters[i].process(*aep);
            filters[i].process(removed, -1.0);
        }

        //add events that need to be added to the out bottle
        //outBottle.addEvent(**qi);

    }

    double respsum = 0.0;
    double totale = 0.0;
    for(int i = 0; i < numberOri; i++) {
        for(int j = 0; j < numberPhases; j++) {
            if(filters[j + i * numberPhases].getResponse() > threshold)
            {
                respsum += filterweights[j + i * numberPhases] * filters[j + i * numberPhases].getResponse();
                totale  += filters[j + i * numberPhases].getResponse();
            }
        }
    }

    respsum = respsum / (numberOri * numberPhases * totale);
//    respsum = respsum / (numberOri * numberPhases * totweights);

    double cvel = 0;
    if(encdriver.isValid() && doVergence)
    {

        enccontrol->getEncoders(encs.data());

        if(encs[5] > 47 && respsum > 0.0) {
            //            std::cout << "Verging too much... " << std::endl;
            respsum = 0;
        }

        if(encs[5] < 5.0 && respsum < 0.0) {
            //            std::cout << "Diverging too much... " << std::endl;
            respsum = 0;
        }

        error_d = respsum - errorPrev;
        errorPrev = respsum;

        cvel = respsum * kp + error_d * kd;
        velcontrol->velocityMove(5, cvel);
    }


    if(gazedriver.isValid()) {
        yarp::os::Bottle &scopebot = scopeOut.prepare();

        yarp::sig::Vector ang(6);
        gazecontrol->getAngles(ang);

        depth = 220 / (2*tan(ang[2] * M_PI / 180.0));

        scopebot.clear();
        scopebot.addDouble(depth);
        scopebot.addInt((int)doVergence);
        scopebot.addInt(countA);
        scopebot.addInt(countR);
        scopeOut.write();
    }

    yarp::os::Bottle &scopefiltersbot = scopeFiltersOut.prepare();
    scopefiltersbot.clear();
    for(int i = 0; i < numberOri; i++) {
        for(int j = 0; j < numberPhases; j++) {
            if(filters[j + i * numberPhases].getResponse() > 0)
                scopefiltersbot.addDouble(filters[j + i * numberPhases].getResponse());
            else
                scopefiltersbot.addDouble(0.0);
        }
    }
    scopefiltersbot.addDouble(respsum);
    scopeFiltersOut.write();

    static int i = 0;
    if(i % 10 == 0 && debugOut.getOutputCount()) {

        yarp::sig::ImageOf< yarp::sig::PixelBgr > &image = debugOut.prepare();
        image.resize(height, width);
        image.zero();

        ev::vQueue curwin = fifoLeft->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            auto v = as_event<AE>(curwin[j]);
            if(!v) continue;

            image(v->y, width - 1 - v->x) = yarp::sig::PixelBgr(255, 255, 0);
        }

        curwin = fifoRight->getSurf();
        for(unsigned int j = 0; j < curwin.size(); j++) {
            auto v = as_event<AE>(curwin[j]);
            if(!v) continue;

            image(v->y, width - 1 - v->x) = yarp::sig::PixelBgr(255, 0, 255);
        }

        debugOut.write();
    }

    //send on the processed events
//    if(strictness)
//        outPort.writeStrict();
//    else
//        outPort.write();

}

void vVergenceManager::startVerging() {

    doVergence = true;
    if(encdriver.isValid()) {
        controlmode->setControlMode(5, VOCAB_CM_VELOCITY);
    }

}
void vVergenceManager::resetVergence() {

    doVergence = false;
    if(encdriver.isValid()) {
        controlmode->setControlMode(5, VOCAB_CM_POSITION);
        poscontrol->positionMove(5, 20);
    }


}

//empty line to make gcc happy
