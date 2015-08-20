
/*
 * Copyright (C) 2014 Istituto Italiano di Tecnologia
 * Author:  oringinal Charles Clercq
 *          edited by Valentina Vasco
 *          edited by Arren Glover
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

#include "vtsOptFlow.hpp"

using yarp::math::operator *;
using yarp::math::outerProduct;

/******************************************************************************/
//vtsOptFlow
/******************************************************************************/
bool vtsOptFlow::configure(yarp::os::ResourceFinder &rf)
{
    /* set the name of the module */
    std::string moduleName = rf.check("name",
                                      yarp::os::Value("vtsOptFlow")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());


    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int sobelSize = rf.check("filterSize", yarp::os::Value(3)).asInt();
    int coverage = rf.check("coverage", yarp::os::Value(50)).asInt();
    int minEvtsInSobel = rf.check("minEvtsThresh", yarp::os::Value(5)).asInt();
    double inlierThreshold = rf.check("inlierThreshold",
                                   yarp::os::Value(0.1)).asDouble();

    vtsofManager = new vtsOptFlowManager(height, width, sobelSize,
                                         minEvtsInSobel, inlierThreshold);
    return vtsofManager->open(moduleName);

}

/******************************************************************************/
bool vtsOptFlow::interruptModule()
{
    vtsofManager->interrupt();
    yarp::os::RFModule::interruptModule();

    return true;
}

/******************************************************************************/
bool vtsOptFlow::close()
{
    vtsofManager->close();
    delete vtsofManager;
    yarp::os::RFModule::close();

    return true;
}

/******************************************************************************/
double vtsOptFlow::getPeriod() {
    return 1.0;
}

/******************************************************************************/
bool vtsOptFlow::updateModule()
{
    return true;
}

/******************************************************************************/
//vtsOptFlowManager
/******************************************************************************/
vtsOptFlowManager::vtsOptFlowManager(int height, int width, int filterSize,
                                     int minEvtsInSobel, double inlierThreshold)
{

    this->height=height;
    this->width=width;

    //ensure sobel size is at least 3 and an odd number
    if(filterSize < 5) filterSize = 3;
    if(!(filterSize % 2)) filterSize--;
    this->fRad = filterSize / 2;
    this->halfCount = pow(filterSize, 2.0) / 2.0 + 0.5;

    this->minEvtsInSobel = std::min((double)minEvtsInSobel, halfCount/2.0+0.5);
    this->inlierThreshold = inlierThreshold;


    //for speed we predefine the mememory for some matricies
    At = yarp::sig::Matrix(3, filterSize * filterSize);
    AtA = yarp::sig::Matrix(3,3);
    abc = yarp::sig::Vector(3);
    A2 = yarp::sig::Matrix(3, 3);

    eventsComputed = 0;
    eventsPotential = 0;
    bottleCount = 0;

    //create our surface in synchronous mode
    surface = new emorph::vSurface(width, height, false);
}

/******************************************************************************/
bool vtsOptFlowManager::open(std::string moduleName)
{
    /*create all ports*/
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);
    return check1 && check2;
}

/******************************************************************************/
void vtsOptFlowManager::close()
{
    /*close ports*/
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    delete surface;

}

/******************************************************************************/
void vtsOptFlowManager::interrupt()
{
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/******************************************************************************/
void vtsOptFlowManager::onRead(emorph::vBottle &inBottle)
{
    //double starttime = yarp::os::Time::now();
    bool computeflow = true;

    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = inBottle.get<emorph::AddressEvent>();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        //if(computeflow && yarp::os::Time::now() - starttime > 0.0009)
        if(getPendingReads())
            computeflow = false;

        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        if(aep->getChannel()) continue;


        surface->addEvent(*aep);

        emorph::FlowEvent ofe;
        if(computeflow) {
            ofe = compute();
            eventsComputed++;
        }
        eventsPotential++;

        if(ofe.getVx() || ofe.getVy()) {
            outBottle.addEvent(ofe);
        } else {
            outBottle.addEvent(*aep);
        }

    }

    bottleCount++;

    if(bottleCount > 2000) {
        std::cout << (int)(eventsComputed *100.0 / eventsPotential);
        std::cout << "% (" << eventsComputed << ")" << std::endl;
        eventsComputed = 0; eventsPotential = 0; bottleCount = 0;
    }

    outPort.write();

//    double threadtime = yarp::os::Time::now() - starttime;
//    if(threadtime > 0.001) {
//        std::cout << "Thread took too long: " << threadtime * 1000 << "ms";
//        std::cout << std::endl;
//    }
}

/******************************************************************************/
emorph::FlowEvent vtsOptFlowManager::compute()
{
    double dtdy = 0, dtdx = 0;

    //get the most recent event
    emorph::AddressEvent * vr = surface->getMostRecent()->getAs<emorph::AddressEvent>();
    emorph::FlowEvent opt_flow(*vr);

    //find the side of this event that has the collection of temporally nearby
    //events
    double bestscore = emorph::vtsHelper::maxStamp()+1; int besti = 0, bestj = 0;
    for(int i = vr->getX()-fRad; i <= vr->getX()+fRad; i+=fRad) {
        for(int j = vr->getY()-fRad; j <= vr->getY()+fRad; j+=fRad) {
            //get the surface around the recent event
            double sobeltsdiff = 0;
            emorph::vQueue subsurf = surface->getSURF(i, j, fRad);
            if(subsurf.size() < minEvtsInSobel) continue;

            for(int k = 0; k < subsurf.size(); k++) {
                if(subsurf[k]->getStamp() > vr->getStamp())
                    sobeltsdiff += vr->getStamp() - subsurf[k]->getStamp() + emorph::vtsHelper::maxStamp();
                else
                    sobeltsdiff += vr->getStamp() - subsurf[k]->getStamp();
            }
            sobeltsdiff /= subsurf.size();
            if(sobeltsdiff < bestscore) {
                bestscore = sobeltsdiff;
                besti = i; bestj = j;
            }
        }
    }

    if(bestscore > emorph::vtsHelper::maxStamp()) return opt_flow;

    //get the surface around the recent event
    //BE CAREFUL HERE BECAUSE WE CHANGE THE TIMESTAMP (IN OUR SURFACE)!!
    //MAKE SURE THERE ARE NO OTHER REPERCUSIONS!!
    emorph::vQueue subsurf = surface->getSURF(besti, bestj, fRad);
    for(int i = 0; i < subsurf.size(); i++) {
        if(subsurf[i]->getStamp() > vr->getStamp())
            subsurf[i]->setStamp(subsurf[i]->getStamp() - emorph::vtsHelper::maxStamp());
    }


    //take only the most recent half of events
    subsurf.sort();
    while(subsurf.size() > halfCount) {
        subsurf.pop_front();
    }
    if(subsurf.size() < minEvtsInSobel) return opt_flow;

    //and compute the flow
    if(computeGrads(subsurf, *vr, dtdy, dtdx)) {
        opt_flow.setVx(dtdx);
        opt_flow.setVy(dtdy);
        opt_flow.setDeath();
    }

    return opt_flow;
}

/******************************************************************************/
bool vtsOptFlowManager::computeGrads(emorph::vQueue &subsurf,
                                     emorph::AddressEvent &cen,
                                     double &dtdy, double &dtdx)
{

    yarp::sig::Matrix A(subsurf.size(), 3);
    yarp::sig::Vector Y(subsurf.size());
    for(int vi = 0; vi < subsurf.size(); vi++) {
        emorph::AddressEvent *v = subsurf[vi]->getAs<emorph::AddressEvent>();
        A(vi, 0) = v->getX();
        A(vi, 1) = v->getY();
        A(vi, 2) = 1;
        Y(vi) = v->getStamp() * emorph::vtsHelper::tstosecs();
    }

    return computeGrads(A, Y, cen.getX(), cen.getY(), cen.getStamp() *
                        emorph::vtsHelper::tstosecs(), dtdy, dtdx);
}

/******************************************************************************/
bool vtsOptFlowManager::computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
                                     double cx, double cy, double cz,
                                     double &dtdy, double &dtdx)
{

    At=A.transposed();
    AtA=At*A;

    double* dataATA=AtA.data();
    double DET=*dataATA*( *(dataATA+8)**(dataATA+4)-*(dataATA+7)**(dataATA+5))-
            *(dataATA+3)*(*(dataATA+8)**(dataATA+1)-*(dataATA+7)**(dataATA+2))+
            *(dataATA+6)*(*(dataATA+5)**(dataATA+1)-*(dataATA+4)**(dataATA+2));
    if(DET < 1) return false;


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

    // calculating the error of the plane...
    double d = -(abc(1)*cx + abc(0)*cy + abc(2)*cz);
    //for each point then compute the distance
    double inliers = 0;
    for(int i = 0; i < A.rows(); i++) {

        //distance in z direction / time direction
        double dp = Y(i) + (abc(1)*A(i, 0) + abc(0)*A(i, 1) + d)/abc(2);
        if(fabs(dp) < inlierThreshold) inliers++;

    }

    if(inliers < minEvtsInSobel) return false;
    dtdx = abc(1); dtdy = abc(0);
    return true;

}
