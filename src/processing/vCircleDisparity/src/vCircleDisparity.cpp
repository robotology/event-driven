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

#include "vCircleDisparity.h"
#include <algorithm>

/**********************************************************/
bool vcdModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCircleDisparity")).asString();
    setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* create the thread and pass pointers to the module parameters */
    circledisparity.open(moduleName, strict);

    return true ;
}

/**********************************************************/
bool vcdModule::interruptModule()
{
    circledisparity.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vcdModule::close()
{
    circledisparity.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vcdModule::updateModule()
{
    return true;
}

/**********************************************************/
double vcdModule::getPeriod()
{
    return 0.5;
}

/**********************************************************/
circleDisparity::circleDisparity()
{

    //here we should initialise the module
    FIFOL.setTemporalSize(100000 * 7.8125);
    FIFOR.setTemporalSize(100000 * 7.8125);

}
/**********************************************************/
bool circleDisparity::open(const std::string &name, bool strict)
{
    //and open the input port

    this->useCallback();
    if(strict) this->setStrict();

    //std::string inPortName = "/" + name + "/vBottle:i";
    if(!yarp::os::BufferedPort<emorph::vBottle>::open("/" + name + "/vBottle:i"))
        return false;

    //std::string outPortName = "/" + name + "/vBottle:o";
    if(!outPort.open("/" + name + "/vBottle:o"))
        return false;

    if(!disparityPort.open("/" + name + "/disp:o"))
        return false;

    return true;
}

/**********************************************************/
void circleDisparity::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    //remember to also deallocate any memory allocated by this class


}

/**********************************************************/
void circleDisparity::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();

}

/**********************************************************/
void circleDisparity::onRead(emorph::vBottle &vBottleIn)
{
    yarp::os::Stamp st;
    this->getEnvelope(st);

    emorph::vBottle &vBottleOut = outPort.prepare();
    vBottleOut = vBottleIn;

    //we just need to get our updated TS
    emorph::vQueue q = vBottleIn.getAll();
    q.sort(true);
    FIFOL.removeEvents(*(q.back()));
    FIFOR.removeEvents(*(q.back()));

    //get the events and see if we can get a ball observation
    q = vBottleIn.getSorted<emorph::ClusterEventGauss>();

    emorph::vQueue fq;
    std::vector<int> xs, ys, rs;
    emorph::ClusterEventGauss circleft;
    emorph::ClusterEventGauss circright;
    emorph::ClusterEventGauss *circc;

    //update the FIFOs
    for(unsigned int i = 0; i < q.size(); i++) {
        emorph::ClusterEventGauss * vc = q[i]->getUnsafe<emorph::ClusterEventGauss>();
        if(vc->getChannel()) {
            FIFOc = &FIFOR;
            circc = &circright;
        } else {
            FIFOc = &FIFOL;
            circc = &circleft;
        }

        FIFOc->addEvent(*q[i]);

        fq = FIFOc->getSurf();
        int n = fq.size();
        xs.resize(n); ys.resize(n); rs.resize(n);
        for(int i = 0; i < n; i++) {
            emorph::ClusterEventGauss *vtw = fq[i]->getUnsafe<emorph::ClusterEventGauss>();
            xs[i] = vtw->getXCog();
            ys[i] = vtw->getYCog();
            rs[i] = vtw->getXSigma2();
            //p_eyez = std::max(p_eyez, (double)vtw->getXSigma2());
        }
        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        std::sort(rs.begin(), rs.end());

        *circc = *vc;
        circc->setXCog(xs[n / 2]);
        circc->setYCog(ys[n / 2]);
        circc->setXSigma2(rs[n/2]);
        circc->setYSigma2(1);
        circc->setID(1);
    }

    if(circleft.getID() && circright.getID()) {
        //we have both a left and right filtered circle position

        //if(abs(circleft.getYCog() - circright.getYCog()) < 5) {
            int disparity = circleft.getYCog() - circright.getYCog();
            //circright.setXSigma2(abs(disparity));
            vBottleOut.addEvent(circleft);
            vBottleOut.addEvent(circright);
        //}
            yarp::os::Bottle &dbot = disparityPort.prepare();
            dbot.clear();
            dbot.addInt(abs(disparity));
            disparityPort.write();
    }


    //write the output
    outPort.setEnvelope(st);
    outPort.write();

    return;

}

//empty line to make gcc happy
