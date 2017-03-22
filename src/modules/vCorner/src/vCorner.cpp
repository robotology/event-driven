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

#include <iomanip>

using namespace ev;

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
    int sobelRad = rf.check("filterSize", yarp::os::Value(3)).asInt();
    int windowRad = rf.check("spatial", yarp::os::Value(3)).asInt();
    int nEvents = rf.check("nEvents", yarp::os::Value(5000)).asInt();
    double thresh = rf.check("thresh", yarp::os::Value(0.08)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    cornermanager = new vCornerManager(height, width, sobelRad, windowRad, nEvents, thresh);
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
vCornerManager::vCornerManager(int height, int width, int sobelRad, int windowRad, int nEvents, double thresh)
{
    this->height = height;
    this->width = width;

    this->sobelRad = sobelRad;
    this->windowRad = windowRad;

    //ensure sobel size is less than length of spatial window
    if(sobelRad > windowRad)
        std::cerr << "ERROR: sobelRad needs to be less than windowRad " << std::endl;

    this->nEvents = nEvents;
    this->thresh = thresh;

    //for speed we predefine the memory for some matricies
    int sobelsize = 2*sobelRad + 1;
    if(!(sobelsize % 2)) std::cout << "WARNING: filter size is not odd " << std::endl;
    sobelx = yarp::sig::Matrix(sobelsize, sobelsize);
    sobely = yarp::sig::Matrix(sobelsize, sobelsize);

    //create sobel filters
    setSobelFilters(sobelsize, sobelx, sobely);

    //create surface representations
    surfaceOfR = new ev::fixedSurface(nEvents, width, height);
    surfaceOnR = new ev::fixedSurface(nEvents, width, height);
    surfaceOfL = new ev::fixedSurface(nEvents, width, height);
    surfaceOnL = new ev::fixedSurface(nEvents, width, height);

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
    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    std::string debugPortName = "/" + moduleName + "/debug:o";
    bool check3 = debugPort.open(debugPortName);

    return check1 && check2 && check3;

}

/**********************************************************/
void vCornerManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();
    debugPort.close();

    delete surfaceOnL;
    delete surfaceOfL;
    delete surfaceOnR;
    delete surfaceOfR;

}

/**********************************************************/
void vCornerManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
    debugPort.interrupt();
}

/**********************************************************/
void vCornerManager::onRead(ev::vBottle &bot)
{
    /*prepare output vBottle*/
    ev::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto aep = is_event<AE>(*qi);
//        if(!aep) continue;

        //add the event to the appropriate surface
        ev::vSurface2 * cSurf;
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

        cSurf->addEvent(aep);

        //detect corner
        bool isc = detectcorner(cSurf);

        //add corner event to the output bottle
        if(isc) {
            auto ce = make_event<LabelledAE>(aep);
            ce->ID = 1;
            outBottle.addEvent(ce);
        }
        else
            outBottle.addEvent(aep);

    }

    if (strictness) outPort.writeStrict();
    else outPort.write();

}

/**********************************************************/

bool vCornerManager::detectcorner(ev::vSurface2 *surf)
{
    double dx = 0.0, dy = 0.0, dxy = 0.0, score = 0.0;
    bool isc;

    //get the most recent event
    auto vr = is_event<AE>(surf->getMostRecent());

    for(int i = vr->x-windowRad; i <= vr->x+windowRad; i+=windowRad) {
        for(int j = vr->y-windowRad; j <= vr->y+windowRad; j+=windowRad) {

            //get the surface around the current event
            const vQueue &subsurf = surf->getSurf(i, j, sobelRad);

            //compute the derivatives
            dx += pow(convSobel(subsurf, sobelx, i, j), 2);
            dy += pow(convSobel(subsurf, sobely, i, j), 2);
            dxy += convSobel(subsurf, sobelx, i, j) * convSobel(subsurf, sobely, i, j);

        }
    }

    //compute score
    score = (dx*dy - dxy*dxy) - 0.04*((dx + dy) * (dx + dy));

    yarp::os::Bottle &scorebottleout = debugPort.prepare();
    scorebottleout.clear();
    scorebottleout.addDouble(score);
    debugPort.write();

    //if score > thresh tag ae as ce
    if(score > thresh) {
        return isc = true;
    }
    else {
        return isc = false;
    }
}
/**********************************************************/
double vCornerManager::convSobel(const ev::vQueue &w, yarp::sig::Matrix &sobel, int a, int b)
{
    double val = 0.0, norm = 0.0;

    //compute the convolution between the filter and the window
    for(unsigned int k = 0; k < w.size(); k++) {

        auto vi = is_event<AE>(w[k]);
        int deltax = a - vi->x + sobelRad;
        int deltay = b - vi->y + sobelRad;
        val += sobel[deltax][deltay];
    }

    for(int i = 0; i < (2*sobelRad+1); i++) {
        for(int j = 0; j < (2*sobelRad+1); j++){
            norm += fabs(sobel[i][j]);
        }
    }

    return (val / norm);

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
