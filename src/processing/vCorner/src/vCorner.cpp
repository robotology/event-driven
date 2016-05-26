/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
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

#include "vCorner.h"

/**********************************************************/
bool vCornerModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCorner")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int sobelsize = rf.check("filterSize", yarp::os::Value(3)).asInt();
    int thickness = rf.check("thick", yarp::os::Value(2)).asInt();
    double thresh = rf.check("thresh", yarp::os::Value(0.05)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    cornermanager = new vCornerManager(height, width, sobelsize, thickness, thresh);
    return cornermanager->open(moduleName, strict);

}

/**********************************************************/
bool vCornerModule::interruptModule()
{
    cornermanager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCornerModule::close()
{
    cornermanager->close();
    delete cornermanager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCornerModule::updateModule()
{
    return true;
}

/**********************************************************/
double vCornerModule::getPeriod()
{
    return 1;
}

/******************************************************************************/
//vCornerManager
/******************************************************************************/
vCornerManager::vCornerManager(int height, int width, int sobelsize, int thickness, double thresh)
{
    this->height = height;
    this->width = width;

    this->sobelsize = sobelsize;
    //ensure sobel size is at least 3 and an odd number
    if(sobelsize < 5) sobelsize = 3;
    if(!(sobelsize % 2)) sobelsize--;

    this->thickness = thickness;

    this->thresh = thresh;
    this->fRad = sobelsize / 2;
    this->minEvts = 3;

    //create our edge
    edge = new emorph::vEdge(width, height);
    edge->setThickness(thickness);

    //for speed we predefine the mememory for some matricies
    sobelx = yarp::sig::Matrix(sobelsize, sobelsize);
    sobely = yarp::sig::Matrix(sobelsize, sobelsize);

    //create sobel filters
    setSobelFilters(sobelsize, sobelx, sobely);

}
/**********************************************************/
bool vCornerManager::open(const std::string moduleName, bool strictness)
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

/**********************************************************/
void vCornerManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();

    delete edge;
}

/**********************************************************/
void vCornerManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/**********************************************************/
void vCornerManager::onRead(emorph::vBottle &bot)
{
    int detectedCorners = 0;

    /*prepare output vBottle with AEs extended with optical flow events*/
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        if(aep->getChannel()) continue;

        edge->addEventToEdge(aep);

        bool isc = detectcorner();

        //THIS COULD BE PROBLEMATIC AS FLOW WILL BE OVERWRITTEN BY IEs
        //THEREFORE THE NEXT MODULE LOSES THE FLOW INFORMATION
        if(isc) {
            emorph::InterestEvent ce = *aep;
            ce.setID(isc);
            outBottle.addEvent(ce);
            detectedCorners++;
//            std::cout << "corners detected in the bottle " << detectedCorners << std::endl;
        }
        else outBottle.addEvent(*aep);
    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

/**********************************************************/
bool vCornerManager::detectcorner() //(const emorph::vQueue &edge)
{
    double dtdx = 0, dtdy = 0,
            dttdx = 0, dttdy = 0, dttdxy = 0, dttdyx = 0,
            //normdx = 0, normdy = 0,
            score = 0;
    bool isc;

    //get the most recent event
    emorph::AddressEvent * vr =
            edge->getMostRecent()->getAs<emorph::AddressEvent>();

    for(int x = vr->getX()-fRad; x <= vr->getX()+fRad; x+=fRad) {
        for(int y = vr->getY()-fRad; y <= vr->getY()+fRad; y+=fRad) {
            //get the surface around the recent event
            const emorph::vQueue &subedge = edge->getSurf(x, y, fRad);

            //we need at least 3 events to say that it's a corner
            if(subedge.size() < minEvts) continue;

            //compute the derivatives
            dtdx = convSobel(subedge, sobelx, x, y);
            dtdy = convSobel(subedge, sobely, x, y);
            int dx = x - vr->getX() + fRad;
            int dy = y - vr->getY() + fRad;
            dttdx += singleSobel(dtdx, sobelx, dx, dy); //normdx += fabs(sobelx[dx][dy]);
            dttdy += singleSobel(dtdy, sobely, dx, dy); //normdy += fabs(sobely[dx][dy]);
            dttdxy += singleSobel(dtdx, sobely, dx, dy);
            dttdyx += singleSobel(dtdy, sobelx, dx, dy);

        }
    } // end for

    dttdx = dttdx / (sobelsize*sobelsize - 1);//normdx;
    dttdy = dttdy / (sobelsize*sobelsize - 1);//normdy;
    dttdxy = dttdxy / (sobelsize*sobelsize - 1);//normdy;
    dttdyx = dttdyx / (sobelsize*sobelsize - 1);//normdx;

//    std::cout << "dxx " << dttdx << std::endl;
//    std::cout << "dyy " << dttdy << std::endl;
//    std::cout << "dxy " << dttdxy << std::endl;
//    std::cout << "dyx " << dttdyx << std::endl;

    //compute score
    score = fabs(dttdx*dttdy - dttdxy*dttdyx) - 0.04*pow((dttdx + dttdy), 2);
//    std::cout << "score " << score << std::endl;

    //if score > thresh tag ae as ce
    if(score > thresh) {
        return isc = true;
    }
    else {
        return isc = false;
    }
}

/**********************************************************/
double vCornerManager::convSobel(const emorph::vQueue &subedge, yarp::sig::Matrix &sobel, int x, int y)
{
    double val = 0; //norm = 0;

    int tsMin = 0;//getMinStamp(subedge);
    int tsMax = getMaxStamp(subedge);

    //compute the sparse convolution between the filter and the window
    //normalized by the minimum timestamp
    for(unsigned int k = 0; k < subedge.size(); k++) {
        int ts = subedge[k]->getStamp();
        int dx = x - subedge[k]->getUnsafe<emorph::AddressEvent>()->getX() + fRad;
        int dy = y - subedge[k]->getUnsafe<emorph::AddressEvent>()->getY() + fRad;
        val += singleSobel(((double)(ts - tsMin) / (tsMax - tsMin)), sobel, dx, dy);
        //norm += fabs(sobel[dx][dy]);
    }

    return(val / (sobelsize*sobelsize - 1));

}

/**********************************************************/
inline double vCornerManager::singleSobel(double val, yarp::sig::Matrix &sobel, int dx, int dy)
{
    return (val * sobel[dx][dy]);
}

/**********************************************************/
int vCornerManager::getMinStamp(const emorph::vQueue &subedge)
{
    int tsMin = subedge[0]->getStamp();
    for(unsigned int k = 0; k < subedge.size(); k++) {
        int t = subedge[k]->getStamp();
        if(t < tsMin)
            tsMin = t;
    }
    return(tsMin);
}

/**********************************************************/
int vCornerManager::getMaxStamp(const emorph::vQueue &subedge)
{
    int tsMax = subedge[0]->getStamp();
    for(unsigned int k = 0; k < subedge.size(); k++) {
        int t = subedge[k]->getStamp();
        if(t > tsMax)
            tsMax = t;
    }
    return(tsMax);
}

/**********************************************************/
void vCornerManager::setSobelFilters(int sobelsize, yarp::sig::Matrix &sobelx, yarp::sig::Matrix &sobely)
{
    yarp::sig::Vector Sx(sobelsize);
    yarp::sig::Vector Dx(sobelsize);
    for(int i = 1; i <= sobelsize; i++)
    {
        Sx(i - 1) = factorial((sobelsize - 1))/((factorial((sobelsize - 1)-(i - 1)))*(factorial(i - 1)));
        Dx(i - 1) = Pasc(i - 1, sobelsize - 2)-Pasc(i - 2,sobelsize - 2);
    }

    sobelx = yarp::math::outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    sobely = sobelx.transposed();
}

int vCornerManager::factorial(int a)
{
    if(a <= 1)
        return 1;
    return a*factorial(a - 1);
}

int vCornerManager::Pasc(int k, int n)
{
    int P;
    if ((k >= 0) && (k <= n))
        P = factorial(n)/(factorial(n-k)*factorial(k));
    else
        P = 0;
    return P;
}

//empty line to make gcc happy
