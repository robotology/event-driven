
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

    vtsofManager = new vtsOptFlowManager(height, width, sobelSize, coverage,
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
vtsOptFlowManager::vtsOptFlowManager(int height, int width, int sobelSize,
                                     int coverage, int minEvtsInSobel,
                                     double inlierThreshold)
{

    this->height=height;
    this->width=width;

    //ensure sobel size is at least 3 and an odd number
    if(sobelSize < 5) sobelSize = 3;
    if(!(sobelSize % 2)) sobelSize--;
    this->sobelSize=sobelSize;
    this->sobRad = sobelSize / 2;
    this->fullCount = pow(sobelSize, 2.0);
    this->halfCount = fullCount / 2.0 + 0.5;
    this->minEvtsInSobel = std::min((double)minEvtsInSobel, halfCount/2.0+0.5);
    this->inlierThreshold = inlierThreshold;


    //for speed we predefine the mememory for some matricies
    sobelx = yarp::sig::Matrix(sobelSize, sobelSize);
    sobely = yarp::sig::Matrix(sobelSize, sobelSize);
    At = yarp::sig::Matrix(3, sobelSize * sobelSize);
    AtA = yarp::sig::Matrix(3,3);
    abc = yarp::sig::Vector(3);
    A2 = yarp::sig::Matrix(3, 3);

    setSobelFilters(sobelSize, sobelx, sobely);

    eventsComputed = 0;
    eventsPotential = 0;
    bottleCount = 0;

    //create our surface in synchronous mode
    surface = new emorph::vSurface(width, height, coverage, false);
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

    //get the surface around the recent event
    emorph::vQueue subsurf = surface->getSURF(sobRad);

    //take only the most recent half of events
    subsurf.sort();
    int halfcount = sobelSize*sobelSize/2;
    while(subsurf.size() > halfcount) {
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


    double dtdy_mean = 0, dtdx_mean = 0;
    int n = 0;
    int rx = vr->getX();
    int ry = vr->getY();

    //top and bottom perimeter
    for(int x = -sobRad; x <= sobRad; x++) {
        for(int y = -sobRad; y <= sobRad; y += sobelSize-1) {
            //get the surface over which to compute a plane
            emorph::vQueue subsurf = surface->getSURF(rx+x, ry+y, sobRad);
            if(subsurf.size() < minEvtsInSobel) continue;

            //compute!
            bool fit = computeGrads(subsurf, *vr, dtdy, dtdx);
            if(fit) {
                dtdy_mean += dtdy;
                dtdx_mean += dtdx;
                n++;
//                dys.push_back(dtdy);
//                dxs.push_back(dtdx);
            }
        }
    }

    //left and right perimeter sans corners already done above
    for(int x = -sobRad; x <= sobRad; x+=sobelSize-1) {
        for(int y = -sobRad+1; y <= sobRad-1; y++) {
            //get the surface over which to compute a plane
            emorph::vQueue subsurf = surface->getSURF(rx+x, ry+y, sobRad);
            if(subsurf.size() < minEvtsInSobel) continue;

            //compute!
            bool fit = computeGrads(subsurf, *vr, dtdy, dtdx);
            if(fit) {
                dtdy_mean += dtdy;
                dtdx_mean += dtdx;
                n++;
//                dys.push_back(dtdy);
//                dxs.push_back(dtdx);
            }
        }
    }



    //if(n > minSobelsInFlow) {

//        double yvariance = 0, xvariance = 0;
//        for(int i = 0; i < dys.size(); i++) {
//            yvariance += pow(dys[i] - dtdy_mean, 2.0);
//            xvariance += pow(dxs[i] - dtdx_mean, 2.0);
//        }
//        double ystd = sqrt(yvariance / dys.size());
//        double xstd = sqrt(xvariance / dxs.size());

//        std::cout << std::max(ystd, xstd) << std::endl;

//        if(std::max(ystd, xstd) > 10000000) return opt_flow;

//        opt_flow.setVx(/*emorph::vtsHelper::tstosecs() * */dtdx_mean / n);
//        opt_flow.setVy(/*emorph::vtsHelper::tstosecs() * */dtdy_mean / n);
//        opt_flow.setDeath();
//    }

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

    return computeGrads(A, Y, cen.getX(), cen.getY(), cen.getStamp() * emorph::vtsHelper::tstosecs(), dtdy, dtdx);
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
    double d = -(abc(0)*cx + abc(1)*cy + abc(2)*cz);

    //for each point then compute the distance
    double c = 0;
    for(int i = 0; i < A.rows(); i++) {

        //distance in z direction / time direction
        double dp = Y(i) + (abc(0)*A(i, 0) + abc(1)*A(i, 1) + d)/abc(2);
        if(fabs(dp) < inlierThreshold) c++;

    }

    if(c < minEvtsInSobel) return false;

//    dtdy = 0; dtdx = 0;
//    for(int i=0; i<sobelSize; i++) {
//        for(int ii=0; ii<sobelSize;ii++)
//        {
//            dtdx+=sobelx(i,ii)*
//                    (abc(0)*((sobelSize-1)-i)+abc(1)*((sobelSize-1)-ii)+abc(2));
//            dtdy+=sobely(i,ii)*
//                    (abc(0)*((sobelSize-1)-i)+abc(1)*((sobelSize-1)-ii)+abc(2));
//        }
//    }

    dtdx = abc(2) / abc(0);
    dtdy = abc(2) / abc(1);
    std::cout << abc.toString() << std::endl;

    //dtdx *= sqrt(fabs(dtdx)) / fabs(dtdx);
    //dtdy *= sqrt(fabs(dtdy)) / fabs(dtdy);

    dtdx /= 1000;//fullCount;//12.0;
    dtdy /= 1000;//fullCount;



    return true;

}

/******************************************************************************/
int factorial(int _v)
{
    if(_v<=1)
        return 1;
    return _v*factorial(_v-1);
}

int Pasc(int k, int n)
{
    int P;
    if ( (k>=0) && (k<=n) )
        P=factorial(n)/(factorial(n-k)*factorial(k));
    else
        P=0;
    return P;
}

void vtsOptFlowManager::setSobelFilters(uint _sz, yarp::sig::Matrix& _sfx, yarp::sig::Matrix& _sfy)
{
    yarp::sig::Vector Sx(_sz);
    yarp::sig::Vector Dx(_sz);
    for(int i=1; i<=_sz; i++)
    {
        Sx(i-1)=factorial((_sz-1))/((factorial((_sz-1)-(i-1)))*(factorial(i-1)));
        Dx(i-1)=Pasc(i-1,_sz-2)-Pasc(i-2,_sz-2);
    }
    _sfx=outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    _sfy=_sfx.transposed();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
