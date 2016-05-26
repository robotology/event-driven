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

using yarp::math::operator *;
using yarp::math::outerProduct;

/******************************************************************************/
//vFlowModule
/******************************************************************************/
bool vFlowModule::configure(yarp::os::ResourceFinder &rf)
{
    /* set the name of the module */
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vFlow")).asString();
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

/******************************************************************************/
bool vFlowModule::interruptModule()
{
    flowmanager->interrupt();
    yarp::os::RFModule::interruptModule();

    return true;
}

/******************************************************************************/
bool vFlowModule::close()
{
    flowmanager->close();
    delete flowmanager;
    yarp::os::RFModule::close();

    return true;
}

/******************************************************************************/
double vFlowModule::getPeriod() {
    return 1.0;
}

/******************************************************************************/
bool vFlowModule::updateModule()
{
    return true;
}

/******************************************************************************/
//vFlowManager
/******************************************************************************/
vFlowManager::vFlowManager(int height, int width, int filterSize,
                                     int minEvtsOnPlane)
{

    this->height=height;
    this->width=width;

    //ensure sobel size is at least 3 and an odd number
    if(filterSize < 5) filterSize = 3;
    if(!(filterSize % 2)) filterSize--;
    this->fRad = filterSize / 2;
    this->halfCount = pow(filterSize, 2.0) / 2.0 + 0.5;
    this->planeSize = pow(filterSize, 2.0);

    this->minEvtsOnPlane = minEvtsOnPlane;

    //for speed we predefine the mememory for some matricies
    At = yarp::sig::Matrix(3, filterSize * filterSize);
    AtA = yarp::sig::Matrix(3,3);
    abc = yarp::sig::Vector(3);
    A2 = yarp::sig::Matrix(3, 3);

    eventsComputed = 0;
    eventsPotential = 0;
    bottleCount = 0;

    //create our surface in synchronous mode
    surfaceOnL = new emorph::temporalSurface(width, height);
    surfaceOfL = new emorph::temporalSurface(width, height);
    surfaceOnR = new emorph::temporalSurface(width, height);
    surfaceOfR = new emorph::temporalSurface(width, height);

}

/******************************************************************************/
bool vFlowManager::open(std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);
    return check1 && check2;
}

/******************************************************************************/
void vFlowManager::close()
{
    /*close ports*/
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    delete surfaceOnL;
    delete surfaceOfL;
    delete surfaceOnR;
    delete surfaceOfR;

}

/******************************************************************************/
void vFlowManager::interrupt()
{
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/******************************************************************************/
void vFlowManager::onRead(emorph::vBottle &inBottle)
{
    //bool computeflow = true;

    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = inBottle.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        //if(aep->getChannel()) continue;

        if(aep->getChannel()) {
            if(aep->getPolarity())
                cSurf = surfaceOfR;
            else
                cSurf = surfaceOnR;
        } else {
            if(aep->getPolarity())
                cSurf = surfaceOfL;
            else
                cSurf = surfaceOnL;
        }

        cSurf->addEvent(*aep);

        emorph::FlowEvent *ofe = compute();
//        eventsComputed++;
//        eventsPotential++;

        if(ofe) outBottle.addEvent(*ofe);
        else outBottle.addEvent(*aep);

    }

//    bottleCount++;

//    if(bottleCount > 2000) {
//        std::cout << (int)(eventsComputed *100.0 / eventsPotential);
//        std::cout << "% (" << eventsComputed << ")" << std::endl;
//        eventsComputed = 0; eventsPotential = 0; bottleCount = 0;
//    }


    if(strictness) outPort.writeStrict();
    else outPort.write();

}

/******************************************************************************/
#define COS135on2 0.38268
emorph::FlowEvent *vFlowManager::compute()
{
    emorph::FlowEvent * vf = NULL;
    double dtdy = 0, dtdx = 0;

    //get the most recent event
    emorph::AddressEvent * vr =
            cSurf->getMostRecent()->getAs<emorph::AddressEvent>();


    //find the side of this event that has the collection of temporally nearby
    //events
    double bestscore = emorph::vtsHelper::maxStamp()+1;
    int besti = 0, bestj = 0;

    for(int i = vr->getX()-fRad; i <= vr->getX()+fRad; i+=fRad) {
        for(int j = vr->getY()-fRad; j <= vr->getY()+fRad; j+=fRad) {
            //get the surface around the recent event
            double sobeltsdiff = 0;
            const emorph::vQueue &subsurf = cSurf->getSurf(i, j, fRad);
            if(subsurf.size() < planeSize) continue;

            for(unsigned int k = 0; k < subsurf.size(); k++) {
                sobeltsdiff += vr->getStamp() - subsurf[k]->getStamp();
                if(subsurf[k]->getStamp() > vr->getStamp()) {
                    //subsurf[k]->setStamp(subsurf[k]->getStamp() -
                    //                     emorph::vtsHelper::maxStamp());
                    sobeltsdiff += emorph::vtsHelper::maxStamp();
                }
            }
            sobeltsdiff /= subsurf.size();
            if(sobeltsdiff < bestscore) {
                bestscore = sobeltsdiff;
                besti = i; bestj = j;
            }
        }
    }

    if(bestscore > emorph::vtsHelper::maxStamp()) return vf;

    const emorph::vQueue &subsurf = cSurf->getSurf(besti, bestj, fRad);

    //and compute the flow
    if(computeGrads(subsurf, *vr, dtdy, dtdx) >= minEvtsOnPlane) {
        vf = new emorph::FlowEvent(*vr);
        vf->setVx(dtdx);
        vf->setVy(dtdy);
        vf->setDeath();
    }
//    if(!vf) {
//        //the continuation

//        double adx = 0, ady = 0;
//        int an = 0;
//        emorph::vQueue::const_iterator qi;
//        for(qi = subsurf.begin(); qi != subsurf.end(); qi++) {

//            emorph::FlowEvent *vf2 = (*qi)->getAs<emorph::FlowEvent>();
//            if(!vf2) continue;

////            adx += vf2->getVx();
////            ady += vf2->getVy();
////            an++;

//            double vx = vf2->getVy(); double vy = vf2->getVx();
//            double mag = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
//            vx /= (mag * COS135on2);
//            vy /= (mag * COS135on2);
//            int dx = 0, dy = 0;
//            if(vx > 1) dx = 1; if(vx < -1) dx = -1;
//            if(vy > 1) dy = 1; if(vy < -1) dy = -1;

//            if(vf2->getX() + dx == vr->getX() && vf2->getY() - dy == vr->getY()
//                    || vf2->getX() - dx == vr->getX() && vf2->getY() + dy == vr->getY()
//                    || vf2->getX() + dx == vr->getX() && vf2->getY() + dy == vr->getY()
//                    ) {
//                adx += vf2->getVx();
//                ady += vf2->getVy();
//                an++;
//            }
//        }

//        if(an > 1) {
//            vf = new emorph::FlowEvent(*vr);
//            vf->setVx(adx / an);
//            vf->setVy(ady / an);
//            vf->setDeath();
//            std::cout << "Upgraded event from " << an << " others" << std::endl;
//        }

//    }

    if(vf) cSurf->addEvent(*vf);

    return vf;
}

/******************************************************************************/
int vFlowManager::computeGrads(const emorph::vQueue &subsurf,
                                     emorph::AddressEvent &cen,
                                     double &dtdy, double &dtdx)
{

    yarp::sig::Matrix A(subsurf.size(), 3);
    yarp::sig::Vector Y(subsurf.size());
    for(unsigned int vi = 0; vi < subsurf.size(); vi++) {
        emorph::AddressEvent *v = subsurf[vi]->getAs<emorph::AddressEvent>();
        A(vi, 0) = v->getX();
        A(vi, 1) = v->getY();
        A(vi, 2) = 1;
        if(v->getStamp() > cen.getStamp()) {
            Y(vi) = (v->getStamp() - emorph::vtsHelper::maxStamp()) * emorph::vtsHelper::tstosecs();
        } else {
            Y(vi) = v->getStamp() * emorph::vtsHelper::tstosecs();
        }
    }

    return computeGrads(A, Y, cen.getX(), cen.getY(), cen.getStamp() *
                        emorph::vtsHelper::tstosecs(), dtdy, dtdx);
}

/******************************************************************************/
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
