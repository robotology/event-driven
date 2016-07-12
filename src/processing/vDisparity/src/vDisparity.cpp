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

    //open rpc port
    if(!rpcOut.open("/" + getName() + "/rpctrigger:i"))
        return false;

    //make the respond method of this RF module respond to the rpcPort
    attach(rpcOut);

    /* create the thread and pass pointers to the module parameters */
    disparityManager = new vDisparityManager(rf.check("width", yarp::os::Value(128)).asInt(),
                                             rf.check("height", yarp::os::Value(128)).asInt(),
                                             rf.check("nEvents", yarp::os::Value(200)).asInt(),
                                             rf.check("ori", yarp::os::Value(1)).asInt(),
                                             rf.check("phases", yarp::os::Value(7)).asInt(),
                                             rf.check("disparity", yarp::os::Value(14)).asInt(),
                                             rf.check("stdsperlambda", yarp::os::Value(6.0)).asDouble());

    return disparityManager->open(getName(), strict);
}

/**********************************************************/
bool vDisparityModule::interruptModule()
{
    rpcOut.interrupt();
    disparityManager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vDisparityModule::close()
{
    rpcOut.close();
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
    if (command.get(0).asString() == "start") {
        reply.addString("Starting Verging...");
        this->disparityManager->startVerging();
   }
    else if (command.get(0).asString() == "reset") {
        reply.addString("Resetting...");
        this->disparityManager->resetVergence();

    }
    return true;
}


/**********************************************************/
vDisparityManager::vDisparityManager(int width, int height, int nEvents, int numberOri, int numberPhases, int maxDisparity, double stdsPerLambda)
{
    this->width = width;
    this->height = height;
    this->winsize = maxDisparity * 8.0 / 3.0;

    doVergence = true;

    //enforce an odd number of phases
    numberPhases = 2 * (numberPhases / 2) + 1;
    double sigma = maxDisparity * 8.0 / (stdsPerLambda * 3.0);

    //create the filterbank
    std::cout << "Initialising Filterbank" << std::endl;
    filters.resize(numberPhases);
    filterweights.resize(numberPhases);
    std::cout << "Phases:";
    for(unsigned int i = 0; i < filters.size(); i++) {
        filters[i].setCenter(width/2, height/2);
        int lambda;
        if(numberPhases == 1)
            lambda = 0;
        else
            lambda = -maxDisparity + i * maxDisparity * 2.0 / (numberPhases - 1) + 0.5;
        filters[i].setParameters(sigma, stdsPerLambda, M_PI * 0.5, lambda);
//        if(i == (filters.size() - 1) / 2)
//            filterweights[i] = 0;
//        else
//            filterweights[i] = lambda / 4.0; // 2.0 / lambda;
        std::cout << " " << lambda;
    }

    std::cout << std::endl;

    //create the filter weights
    //this needs to be more robust (i.e. use filter disparity to set weight)
    std::cout << "Weights: ";
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

        std::cout << filterweights[i] << " ";
    }
    std::cout << std::endl;

    //create the surface representations
    fifoLeft = new emorph::fixedSurface(nEvents, width, height);
    fifoRight = new emorph::fixedSurface(nEvents, width, height);

    gazecontrol = 0;
    enccontrol = 0;
    poscontrol = 0;
    velcontrol = 0;
//    desiredvergence = 20;

    depth = 0;
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
void vDisparityManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
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
void vDisparityManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    scopeOut.interrupt();
    scopeFiltersOut.interrupt();
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

    double respsum = 0.0;
    for(unsigned int i = 0; i < filters.size(); i++) {
        if(filters[i].getResponse() > 0)
            respsum += filterweights[i] * filters[i].getResponse();
    }


//    if(gazedriver.isValid())
//    {
//        yarp::sig::Vector angles(3);
//        gazecontrol->getAngles(angles);
//        //std::cout << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;
//        if(respsum > 100) {
//            std::cout << "Trying to move: ";
//            angles(2) = angles(2) + 1;
//            std::cout << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;
//            yarp::sig::Vector pose(3); pose[0] = -1; pose[1] = 0.5; pose[2] = 0.4;
//            gazecontrol->lookAtFixationPoint(pose);
//            //if(!gazecontrol->lookAtAbsAngles(angles))
//                //std::cout << "Error moving" << std::endl;
//        } else if(respsum < -100) {
//            angles(2) = angles(2) - 1;
//            if(!gazecontrol->lookAtAbsAngles(angles))
//                std::cout << "Error moving" << std::endl;
//        }
//    }

//    //static int i = 0;
//    if(encdriver.isValid())
//    {

//        enccontrol->getEncoders(encs.data());

//        desiredvergence  = encs[5] + respsum / 100;

//        if(abs(respsum) > 50) {
//            std::cout << "Trying to move to : " << desiredvergence << std::endl;
//            poscontrol->setRefSpeed(5, 100);
//            if(poscontrol->positionMove(5, desiredvergence))
//                std::cout << "Motion done " << std::endl;
//            //bool motiondone = false;
////            while(!motiondone)
////                poscontrol->checkMotionDone(&motiondone);

//        }
//        //        if(respsum > 120) {
////            std::cout << "Trying to verge in: ";
////            std::cout << encs[5] + 1 << std::endl;
////            if(poscontrol->positionMove(5, encs[5] + 1))
////                std::cout << "Verged in " << std::endl;

////        }
////        else if(respsum < -120) {
////            std::cout << "Trying to verge out: ";
////            std::cout << encs[5] - 1 << std::endl;
////            if(poscontrol->positionMove(5, encs[5] - 1))
////                std::cout << "Verged out " << std::endl;
////        }
//    }


//    yarp::sig::Vector fp(3);
//    yarp::sig::Vector ang(6);
    double kp = 0.2;

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

        double cvel = respsum * kp;
//        if(fabs(cvel) < 10) {
//            cvel = cvel / fabs(cvel) * 10;
//        }
        if(velcontrol->velocityMove(5, cvel)) {
            //            std::cout << "Moving at " << respsum * kp << std::endl;
            //            gazecontrol->getAngles(ang);
            //            gazecontrol->get3DPointFromAngles(0, ang, fp);


            //            std::cout << fp(0) * 1000 << " " << fp(1) * 100 << " " << fp(2) * 100 << std::endl;

        }

        //        if(abs(respsum) > 10) {

        //            std::cout << "Controlling velocity to " << respsum * kp << std::endl;
        //            if(velcontrol->velocityMove(5, respsum * kp))
        //                std::cout << "Moving " << std::endl;
        //        }
        //        else
        //            velcontrol->velocityMove(5, 0);

        //        if(respsum > 20) {

        //            std::cout << "Controlling velocity to : 50 " << std::endl;
        //            if(velcontrol->velocityMove(5, 50))
        //                std::cout << "Moving" << std::endl;
        //        }
        //        else if(respsum < -20) {

        //            std::cout << "Controlling velocity : -50 " << std::endl;
        //            if(velcontrol->velocityMove(5, -50))
        //                std::cout << "Moving" << std::endl;
        //        }
        //        else
        //            velcontrol->velocityMove(5, 0);

    }

    if(gazedriver.isValid()) {
        yarp::os::Bottle &scopebot = scopeOut.prepare();
        //gazecontrol->getFixationPoint(fp);

        yarp::sig::Vector ang(6);
        gazecontrol->getAngles(ang);
//        std::cout << ang.toString() << std::endl;

        depth = 220 / (2*tan(ang[2] * M_PI / 180.0));

        //depth = -fp(0) * 1000;
        scopebot.clear();
        scopebot.addDouble(depth);
        scopebot.addInt((int)doVergence);
        scopeOut.write();
    }

    yarp::os::Bottle &scopefiltersbot = scopeFiltersOut.prepare();
    scopefiltersbot.clear();
    for(unsigned int i = 0; i < filters.size(); i++) {

        //respsum += filterweights[i] * filters[i].getResponse();
        if(filters[i].getResponse() > 0)
            scopefiltersbot.addDouble(filters[i].getResponse());
        else
            scopefiltersbot.addDouble(0.0);
    }
    scopefiltersbot.addDouble(respsum);
    scopeFiltersOut.write();

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

void vDisparityManager::startVerging() {

    doVergence = true;
    if(encdriver.isValid()) {
        controlmode->setControlMode(5, VOCAB_CM_VELOCITY);
    }

}
void vDisparityManager::resetVergence() {

    doVergence = false;
    if(encdriver.isValid()) {
        controlmode->setControlMode(5, VOCAB_CM_POSITION);
        poscontrol->positionMove(5, 20);
    }


}

//empty line to make gcc happy
