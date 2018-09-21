/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "module.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        std::cout << "Could not connect to YARP" << std::endl;
        return false;
    }

    /* create the module */
    module instance;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "vDelayControl.ini" );
    rf.configure( argc, argv );

    return instance.runModule(rf);
}


/*////////////////////////////////////////////////////////////////////////////*/
//vParticleModule
/*////////////////////////////////////////////////////////////////////////////*/
bool module::configure(yarp::os::ResourceFinder &rf)
{
    //module name and control
    setName((rf.check("name", yarp::os::Value("/delayControl")).asString()).c_str());
    if(!rpcPort.open(getName() + "/cmd")) {
        yError() << "Could not open rpc port for" << getName();
        return false;
    }
    attach(rpcPort);

    //administrative options
    int nthread = rf.check("threads", yarp::os::Value(1)).asInt();
    int height = rf.check("height", yarp::os::Value(240)).asInt();
    int width = rf.check("width", yarp::os::Value(304)).asInt();
    int bins = rf.check("bins", yarp::os::Value(64)).asInt();
    //int maxq = rf.check("maxq", yarp::os::Value(500)).asInt();
    double gain = rf.check("gain", yarp::os::Value(0.0005)).asDouble();
    int mindelay = rf.check("mindelay", yarp::os::Value(1)).asInt();
    int qlimit = rf.check("qlimit", yarp::os::Value(0)).asInt();
    if(qlimit < 0) qlimit = 0;

    //flags
    bool adaptivesampling = rf.check("adaptive") &&
            rf.check("adaptive", yarp::os::Value(true)).asBool();

    //filter paramters
    int particles = rf.check("particles", yarp::os::Value(100)).asInt();
    double nRandResample = rf.check("randoms", yarp::os::Value(0.0)).asDouble();

    yarp::os::Bottle * seed = rf.find("seed").asList();

    //observation parameters
    double minlikelihood = rf.check("obsthresh", yarp::os::Value(0.2)).asDouble();
    double inlierParameter = rf.check("obsinlier", yarp::os::Value(1.5)).asDouble();
    double particleVariance = rf.check("variance", yarp::os::Value(0.7)).asDouble();
    double trueDetectionThreshold = rf.check("truethresh", yarp::os::Value(0.35)).asDouble();
    double resetTimeout = rf.check("reset", yarp::os::Value(1.0)).asDouble();
    double negativeBias = rf.check("negbias", yarp::os::Value(10.0)).asDouble();

    delaycontrol.setGain(gain);
    delaycontrol.setMaxRawLikelihood(bins);
    delaycontrol.setMinToProc(mindelay);
    delaycontrol.setTrueThreshold(trueDetectionThreshold);
    delaycontrol.setResetTimeout(resetTimeout);
    delaycontrol.setMotionVariance(particleVariance);
    //delaycontrol.setMinRawLikelihood(minlikelihood);

    delaycontrol.initFilter(width, height, particles, bins, adaptivesampling,
                            nthread, minlikelihood, inlierParameter, nRandResample, negativeBias);
    if(seed && seed->size() == 3) {
        yInfo() << "Setting initial seed state:" << seed->toString();
        delaycontrol.setFilterInitialState(seed->get(0).asDouble(), seed->get(1).asDouble(), seed->get(2).asDouble());
    }
    if(!scopePort.open(getName() + "/scope:o")) {
        yError() << "Could not open scope port";
        return false;
    }
    if(!delaycontrol.open(getName(), qlimit))
        return false;
    return delaycontrol.start();

}

/******************************************************************************/
bool module::interruptModule()
{
    delaycontrol.stop();
    return true;
}

/******************************************************************************/
bool module::close()
{

    return true;
}

/******************************************************************************/
bool module::updateModule()
{
    if(scopePort.getOutputCount()) {
        scopePort.prepare() = delaycontrol.getTrackingStats();
        scopePort.write();
    }

    return true;
}

/******************************************************************************/
double module::getPeriod()
{
    return 0.05;
}

#define CMD_HELP  createVocab('h', 'e', 'l', 'p')
#define CMD_SET   createVocab('s', 'e', 't')
#define CMD_RESET createVocab('r', 'e', 's')

bool module::respond(const yarp::os::Bottle& command,
                                yarp::os::Bottle& reply) {

    //initialise for default response
    bool error = false;
    yInfo() << command.size();
    reply.clear();

    //switch on the command word
    switch(command.get(0).asVocab()) {

    case CMD_HELP:
    {
        reply.addString("<<Event-based Particle Filter with Delay Control>>");
        reply.addString("Set the following parameters with | set <param> "
                        "<value> |");
        reply.addString("trackThresh [0-1]");
        reply.addString("trueThresh [0-1]");
        reply.addString("gain [0-1]");
        reply.addString("minToProc [0-inf]");
        reply.addString("resetTimeout [0 inf]");
        reply.addString("negativeBias [0 inf]");
        reply.addString("motionVar [0 inf]");
        reply.addString("inlierParam [0 inf]");
        reply.addString("adaptive [true false]");
        break;
    }
    case CMD_SET:
    {

        std::string param = command.get(1).asString();
        double value = command.get(2).asDouble();

        if(param == "trackThresh") {
            reply.addString("setting tracking parameter");
            delaycontrol.setMinRawLikelihood(value);
        }
        else if(param == "gain") {
            reply.addString("setting delay-control gain");
            delaycontrol.setGain(value);
        }
        else if(param == "trueThresh") {
            reply.addString("setting true classification parameter");
            delaycontrol.setTrueThreshold(value);
        }
        else if(param == "minToProc") {
            reply.addString("setting minimum events per update");
            delaycontrol.setMinToProc(value);
        }
        else if(param == "resetTimeout") {
            reply.addString("setting the particle reset timeout");
            delaycontrol.setResetTimeout(value);
        }
        else if(param == "negativeBias") {
            reply.addString("setting the observation negative bias");
            delaycontrol.setNegativeBias(value);
        }
        else if(param == "motionVar") {
            reply.addString("setting particle motion variance");;
            delaycontrol.setMotionVariance(value);
        }
        else if(param == "inlierParam") {
            reply.addString("setting the inlier width");
            delaycontrol.setInlierParameter(value);
        }
        else if(param == "adaptive") {
            if(value) {
                reply.addString("setting the resample method = adaptive");
                delaycontrol.setAdaptive();
            }
            else {
                reply.addString("setting the resample method = every update");
                delaycontrol.setAdaptive(false);
            }
        }
        else {
            error = true;
            reply.addString("incorrect parameter");
        }
        break;
    }
    case CMD_RESET:
    {
        reply.addString("resetting particle positions");
        delaycontrol.performReset();
        break;
    }
    default:
    {
        error = true;
        break;
    }

    } //switch

    //return the error - the reply is automatically sent
    return !error;

}

