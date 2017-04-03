/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include "vFlow.h"
#include <yarp/os/all.h>

using yarp::math::operator *;
using yarp::math::outerProduct;

using namespace ev;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()) {
        yError("unable to find YARP server!");
        return 1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("vFlow.ini");
    rf.setDefaultContext("eventdriven");
    rf.configure(argc, argv);

    /* instantiate the module */
    vFlowModule module;
    return module.runModule(rf);
}

/******************************************************************************/
//vFlowManager
/******************************************************************************/

void vFlowManager::onRead(ev::vBottle &inBottle)
{

    /*prepare output vBottle with AEs extended with optical flow events*/
    ev::vBottle * outBottle = 0;

    /*get the event queue in the vBottle bot*/
    vQueue q = inBottle.get<AE>();

    for(vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto aep = is_event<AE>(*qi);

        //add the event to the appropriate surface
        vSurface2 * cSurf;
        if(aep->getChannel()) {
            if(aep->polarity)
                cSurf = surfaceOfR;
            else
                cSurf = surfaceOnR;
        } else {
            if(aep->polarity)
                cSurf = surfaceOfL;
            else
                cSurf = surfaceOnL;
        }

        //compute the flow
        cSurf->fastAddEvent(aep);
        double vx, vy;
        if(compute(cSurf, vx, vy)) {
            //successfully computed a flow event
            auto vf = make_event<FlowEvent>(aep);
            vf->vx = vx;
            vf->vy = vy;
            if(!outBottle) {
                outBottle = &outPort.prepare();
                outBottle->clear();
            }
            outBottle->addEvent(vf);
        }
    }

    if(outBottle) {
        yarp::os::Stamp st;
        this->getEnvelope(st); outPort.setEnvelope(st);
        if(strictness) outPort.writeStrict();
        else outPort.write();
    }
}

vFlowManager::vFlowManager(int height, int width, int filterSize,
                                     int minEvtsOnPlane)
{
    //ensure sobel size is at least 3 and an odd number
    if(filterSize < 5) filterSize = 3;
    if(!(filterSize % 2)) filterSize--;
    this->fRad = filterSize / 2;
    this->planeSize = pow(filterSize, 2.0);

    this->minEvtsOnPlane = minEvtsOnPlane;

    //for speed we predefine the mememory for some matricies
    At = yarp::sig::Matrix(3, filterSize * filterSize);
    AtA = yarp::sig::Matrix(3,3);
    abc = yarp::sig::Vector(3);
    A2 = yarp::sig::Matrix(3, 3);


    //create our surface in synchronous mode
    surfaceOnL = new ev::temporalSurface(width, height);
    surfaceOfL = new ev::temporalSurface(width, height);
    surfaceOnR = new ev::temporalSurface(width, height);
    surfaceOfR = new ev::temporalSurface(width, height);
}

bool vFlowManager::open(std::string moduleName, bool strictness)
{
    //set strict mode if necessary
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }

    //open the input port
    this->useCallback(); //we need callback to use the onRead() function
    if(!yarp::os::BufferedPort<ev::vBottle>::open(moduleName + "/vBottle:i"))
        return false;

    //open the output port
    if(!outPort.open(moduleName + "/vBottle:o"))
        return false;

    return true;
}

void vFlowManager::close()
{
    /*close ports*/
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    delete surfaceOnL;
    delete surfaceOfL;
    delete surfaceOnR;
    delete surfaceOfR;

}

void vFlowManager::interrupt()
{
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

bool vFlowManager::compute(ev::vSurface2 *surf, double &vx, double &vy)
{

    //get the most recent event
    auto vr = is_event<AE>(surf->getMostRecent());


    //find the side of this event that has the collection of temporally nearby
    //events. Heuristically more likely to be the correct plane.
    double bestscore = ev::vtsHelper::max_stamp+1;
    int besti = 0, bestj = 0;

    for(int i = vr->x-fRad; i <= vr->x+fRad; i+=fRad) {
        for(int j = vr->y-fRad; j <= vr->y+fRad; j+=fRad) {
            //get the surface around the recent event
            double sobeltsdiff = 0;
            const vQueue subsurf = surf->getSurf(i, j, fRad);
            if(subsurf.size() < planeSize) continue;

            for(unsigned int k = 0; k < subsurf.size(); k++) {
                sobeltsdiff += vr->stamp - subsurf[k]->stamp;
                if(subsurf[k]->stamp > vr->stamp) {
                    //subsurf[k]->setStamp(subsurf[k]->stamp -
                    //                     eventdriven::vtsHelper::maxStamp());
                    sobeltsdiff += ev::vtsHelper::max_stamp;
                }
            }
            sobeltsdiff /= subsurf.size();
            if(sobeltsdiff < bestscore) {
                bestscore = sobeltsdiff;
                besti = i; bestj = j;
            }
        }
    }
    //return if we don't find a good candidate plane
    if(bestscore > ev::vtsHelper::max_stamp) return false;


    //get the events
    const vQueue &subsurf = surf->getSurf(besti, bestj, fRad);
    //const vQueue &subsurf = surf->getSurf(vr->x, vr->y, fRad);

    //and compute the gradients of the plane
    if(computeGrads(subsurf, vr, vy, vx) < minEvtsOnPlane)
        return false;

    return true;
}

int vFlowManager::computeGrads(const ev::vQueue &subsurf,
                                     event<ev::AddressEvent> cen,
                                     double &dtdy, double &dtdx)
{

    yarp::sig::Matrix A(subsurf.size(), 3);
    yarp::sig::Vector Y(subsurf.size());
    for(unsigned int vi = 0; vi < subsurf.size(); vi++) {
        event<ev::AddressEvent> v = as_event<ev::AddressEvent>(subsurf[vi]);
        A(vi, 0) = v->x;
        A(vi, 1) = v->y;
        A(vi, 2) = 1;
        if(v->stamp > cen->stamp) {
            Y(vi) = (v->stamp - ev::vtsHelper::max_stamp) * ev::vtsHelper::tstosecs();
        } else {
            Y(vi) = v->stamp * ev::vtsHelper::tstosecs();
        }
    }

    return computeGrads(A, Y, cen->x, cen->y, cen->stamp *
                        ev::vtsHelper::tstosecs(), dtdy, dtdx);
}

int vFlowManager::computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
                                     double cx, double cy, double cz,
                                     double &dtdy, double &dtdx)
{

    At=A.transposed();
    AtA=At*A;

    double* dataATA=AtA.data();
    double DET=*dataATA*( *(dataATA+8)**(dataATA+4)-*(dataATA+7)**(dataATA+5))-
            *(dataATA+3)*(*(dataATA+8)**(dataATA+1)-*(dataATA+7)**(dataATA+2))+
            *(dataATA+6)*(*(dataATA+5)**(dataATA+1)-*(dataATA+4)**(dataATA+2));
    if(DET < 1) return 0;


    double *dataA=A2.data();
    DET=1.0/DET;
    *(dataA+0)=DET*(*(dataATA+8)**(dataATA+4)-*(dataATA+7)**(dataATA+5));
    *(dataA+1)=DET*(*(dataATA+7)**(dataATA+2)-*(dataATA+8)**(dataATA+1));
    *(dataA+2)=DET*(*(dataATA+5)**(dataATA+1)-*(dataATA+4)**(dataATA+2));
    *(dataA+3)=DET*(*(dataATA+6)**(dataATA+5)-*(dataATA+8)**(dataATA+3));
    *(dataA+4)=DET*(*(dataATA+8)**(dataATA+0)-*(dataATA+6)**(dataATA+2));
    *(dataA+5)=DET*(*(dataATA+3)**(dataATA+2)-*(dataATA+5)**(dataATA+0));
    *(dataA+6)=DET*(*(dataATA+7)**(dataATA+3)-*(dataATA+6)**(dataATA+4));
    *(dataA+7)=DET*(*(dataATA+6)**(dataATA+1)-*(dataATA+7)**(dataATA+0));
    *(dataA+8)=DET*(*(dataATA+4)**(dataATA+0)-*(dataATA+3)**(dataATA+1));

    abc=A2*At*Y;

    double dtdp = sqrt(pow(abc(0), 2.0) + pow(abc(1), 2.0));
    int inliers = 0;
    for(int i = 0; i < A.rows(); i++) {
        //so I think that abc(0) and abc(1) are already scaled to the magnitude
        //of the slope of the plane. E.g. when only using abc(0) and abc(1) and
        //fitting a 3-point plane we always get 0 error. Therefore the differ-
        //ence in time is perfect with only abc(0,1) and the speed should also
        //be.
        double planedt = (abc(0) * (A(i, 0) - cx) + abc(1) * (A(i, 1) - cy));
        double actualdt =  Y(i) - cz;
        if(fabs(planedt - actualdt) < dtdp/2) inliers++;

    }

    double speed = 1.0 / dtdp;

    double angle = atan2(abc(0), abc(1));
    dtdx = speed * cos(angle);
    dtdy = speed * sin(angle);


    return inliers;
}

/******************************************************************************/
//vFlowModule
/******************************************************************************/
bool vFlowModule::configure(yarp::os::ResourceFinder &rf)
{
    /* set the name of the module */
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("/vFlow")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int sobelSize = rf.check("filterSize", yarp::os::Value(3)).asInt();
    int minEvtsOnPlane = rf.check("minEvtsThresh", yarp::os::Value(5)).asInt();

    flowmanager = new vFlowManager(height, width, sobelSize, minEvtsOnPlane);
    return flowmanager->open(moduleName, strict);

}

bool vFlowModule::interruptModule()
{
    flowmanager->interrupt();
    yarp::os::RFModule::interruptModule();

    return true;
}

bool vFlowModule::close()
{
    flowmanager->close();
    delete flowmanager;
    yarp::os::RFModule::close();

    return true;
}

double vFlowModule::getPeriod() {
    return 1.0;
}

bool vFlowModule::updateModule()
{
    return true;
}

