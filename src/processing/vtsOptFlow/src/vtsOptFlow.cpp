
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
    int tWindow = rf.check("tWindow", yarp::os::Value(50000)).asInt();
    int minEvtsInSobel = rf.check("minEvtsThresh", yarp::os::Value(5)).asInt();
    int minSobelsInFlow = rf.check("minSobelsThresh",
                                   yarp::os::Value(3)).asInt();

    vtsofManager = new vtsOptFlowManager(height, width, sobelSize, tWindow,
                                         minEvtsInSobel, minSobelsInFlow);
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
                                     int temporalWindow, int minEvtsInSobel,
                                     int minSobelsInFlow)
{

    this->height=height;
    this->width=width;

    //ensure sobel size is at least 3 and an odd number
    if(sobelSize < 5) sobelSize = 3;
    if(!(sobelSize % 2)) sobelSize--;
    this->sobelSize=sobelSize;
    this->sobRad = sobelSize / 2;
    this->minEvtsInSobel = minEvtsInSobel;
    this->minSobelsInFlow = minSobelsInFlow;

    //for speed we predefine the mememory for some matricies
    sobelx = yarp::sig::Matrix(sobelSize, sobelSize);
    sobely = yarp::sig::Matrix(sobelSize, sobelSize);
    At = yarp::sig::Matrix(3, sobelSize * sobelSize);
    AtA = yarp::sig::Matrix(3,3);
    abc = yarp::sig::Vector(3);
    A2 = yarp::sig::Matrix(3, 3);

    setSobelFilters(sobelSize, sobelx, sobely);

    //create our surface in synchronous mode
    surface = new emorph::vWindow(width, height, temporalWindow, false);
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
    double starttime = yarp::os::Time::now();
    bool computeflow = true;

    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = inBottle.getAll();

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        if(computeflow && yarp::os::Time::now() - starttime > 0.0008)
            computeflow = false;

        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        if(aep->getChannel()) continue;


        surface->addEvent(*aep);

        emorph::OpticalFlowEvent ofe;
        if(computeflow) ofe = compute();

        if(ofe.getVx() || ofe.getVy()) {
            outBottle.addEvent(ofe);
        } else {
            outBottle.addEvent(*aep);
        }

    }

    outPort.write();

    double threadtime = yarp::os::Time::now() - starttime;
    if(threadtime > 0.001) {
        std::cout << "Thread took too long: " << threadtime * 1000 << "ms";
        std::cout << std::endl;
    }
}

/******************************************************************************/
emorph::OpticalFlowEvent vtsOptFlowManager::compute()
{
    emorph::AddressEvent * vr = surface->getMostRecent()->getAs<emorph::AddressEvent>();
    int rx = vr->getX();
    int ry = vr->getY();
    int rp = vr->getPolarity();

    //we compute multiple gradients around a perimeter of the central position
    double dtdy_mean = 0, dtdx_mean = 0;
    double dtdy = 0, dtdx = 0;
    int n = 0;

    //top and bottom perimeter
    for(int x = -sobRad; x <= sobRad; x++) {
        for(int y = -sobRad; y <= sobRad; y += sobelSize-1) {
            //get the surface over which to compute a plane
            emorph::vQueue subsurf = surface->getSURF(rx+x, ry+y, sobRad, rp);
            if(subsurf.size() < minEvtsInSobel) continue;

            //compute!
            bool fit = computeGrads(subsurf, dtdy, dtdx);
            if(fit) {
                dtdy_mean += dtdy;
                dtdx_mean += dtdx;
                n++;
            }
        }
    }

    //left and right perimeter sans corners already done above
    for(int x = -sobRad; x <= sobRad; x+=sobelSize-1) {
        for(int y = -sobRad+1; y <= sobRad-1; y++) {
            //get the surface over which to compute a plane
            emorph::vQueue subsurf = surface->getSURF(rx+x, ry+y, sobRad, rp);
            if(subsurf.size() < minEvtsInSobel) continue;

            //compute!
            bool fit = computeGrads(subsurf, dtdy, dtdx);
            if(fit) {
                dtdy_mean += dtdy;
                dtdx_mean += dtdx;
                n++;
            }
        }
    }

    emorph::OpticalFlowEvent opt_flow(*vr);
    if(n > minSobelsInFlow) {
        //opt_flow.setChannel(vr->getChannel());
        //opt_flow.setPolarity(vr->getPolarity());
        //opt_flow.setX(ry);
        //opt_flow.setY(rx);
        opt_flow.setVx(emorph::vtsHelper::tstosecs() * dtdx_mean / n);
        opt_flow.setVy(emorph::vtsHelper::tstosecs() * dtdy_mean / n);
    }

    return opt_flow;

}

/******************************************************************************/
bool vtsOptFlowManager::computeGrads(emorph::vQueue &subsurf,
                                     double &dtdy, double &dtdx)
{

    yarp::sig::Matrix A(subsurf.size(), 3);
    yarp::sig::Vector Y(subsurf.size());
    for(int vi = 0; vi < subsurf.size(); vi++) {
        emorph::AddressEvent *v = subsurf[vi]->getAs<emorph::AddressEvent>();
        A(vi, 0) = v->getX();
        A(vi, 1) = v->getY();
        A(vi, 2) = 1;
        Y(vi) = v->getStamp();
    }

    return computeGrads(A, Y, dtdy, dtdx);
}

/******************************************************************************/
bool vtsOptFlowManager::computeGrads(yarp::sig::Matrix &A, yarp::sig::Vector &Y,
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
    DET=1/DET;
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

    dtdy = 0; dtdx = 0;
    for(int i=0; i<sobelSize; i++) {
        for(int ii=0; ii<sobelSize;ii++)
        {
            dtdx+=sobelx(i,ii)*
                    (abc(0)*((sobelSize-1)-i)+abc(1)*((sobelSize-1)-ii)+abc(2));
            dtdy+=sobely(i,ii)*
                    (abc(0)*((sobelSize-1)-i)+abc(1)*((sobelSize-1)-ii)+abc(2));
        }
    }

    //I don't know why Charles did this division
    dtdx /= 12.0;
    dtdy /= 12.0;

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
    _sfy=outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    _sfx=_sfy.transposed();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
