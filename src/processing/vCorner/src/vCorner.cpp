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
    int sobelsize = rf.check("filterSize", yarp::os::Value(5)).asInt();
    int windowRad = rf.check("spatial", yarp::os::Value(5)).asInt();
    int sigma = rf.check("sigma", yarp::os::Value(1.0)).asDouble();
    int nEvents = rf.check("nEvents", yarp::os::Value(5000)).asInt();
    double thresh = rf.check("thresh", yarp::os::Value(8)).asDouble();

    /* create the thread and pass pointers to the module parameters */
    cornermanager = new vCornerManager(height, width, sobelsize, windowRad, sigma, nEvents, thresh);
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
vCornerManager::vCornerManager(int height, int width, int sobelsize, int windowRad, double sigma, int nEvents, double thresh)
{
    this->height = height;
    this->width = width;

    this->sobelsize = sobelsize;
    this->sobelrad = (sobelsize - 1)/2;
    this->windowRad = windowRad;

    //ensure that sobel size is an odd number
    if(!(sobelsize % 2))
    {
        std::cout << "Warning: sobelsize should be odd" << std::endl;
        sobelsize--;
        std::cout << "sobelsize = " << sobelsize << " will be used" << std::endl;
    }
    this->nEvents = nEvents;
    this->thresh = thresh;

    this->gaussiansize = 2*windowRad + 2 - sobelsize;
    //for speed we predefine the memory for some matricies
    sobel.setSobelFilters(sobelsize);
//    sobel.printFilters();

//    sobelx = yarp::sig::Matrix(sobelsize, sobelsize);
//    sobely = yarp::sig::Matrix(sobelsize, sobelsize);
    gaussian = yarp::sig::Matrix(gaussiansize, gaussiansize);

    //create sobel filters
//    setSobelFilters(sobelx, sobely);
    setGaussianFilter(sigma, gaussian);

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

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

    std::string debugPortName = "/" + moduleName + "/score:o";
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

    //get the most recent event
    auto vc = is_event<AE>(surf->getMostRecent());
    int currx = vc->x;
    int curry = vc->y;

    //length of the result of the convolution
    int l = windowRad - sobelrad;

    //get the queue of events
    const vQueue &subsurf = surf->getSurf(vc->x, vc->y, windowRad);

    //update filter response
    for(unsigned int i = 0; i < subsurf.size(); i++)
    {
        //events are in the surface
        auto vi = is_event<AE>(subsurf[i]);
        int x = vi->x;
        int y = vi->y;    

//        std::cout << i << " " << "(" << x << "," << y << ")" << " (" << cx << "," << cy << ")" <<  std::endl;

        for(int cx = currx-l; cx <= currx+l; cx++)
        {
            for(int cy = curry-l; cy <= curry+l; cy++)
            {
                if(abs(x - cx) <= sobelrad && abs(y - cy) <= sobelrad)
                {
                    //(cx,cy) is the pixel where we apply the sobel filter
                    sobel.setCenter(cx, cy);
                    sobel.process(*vi, currx, curry);
//                    sobel.updateresponse(*vi, *vc);
                }
            }
        }
    }

    //get the final response
    for(int m = 0; m < 2*l+1; m++)
    {
        for(int n = 0; n < 2*l+1; n++)
        {
            double sx = sobel.getResponseX(m, n);
            double sy = sobel.getResponseY(m, n);
            double g = gaussian(m, n);
//            std::cout << m << " " << n << " " << sx << " " << sy << std::endl;
            dx += g * pow(sx, 2);
            dy += g * pow(sy, 2);
            dxy += g * sx * sy;
        }
    }

    //reset responses
    sobel.resetResponses();

    //compute score
    score = (dx*dy - dxy*dxy) - 0.04*((dx + dy) * (dx + dy));

    if(debugPort.getOutputCount()) {
        yarp::os::Bottle &scorebottleout = debugPort.prepare();
        scorebottleout.clear();
        scorebottleout.addDouble(score);
        debugPort.write();
    }

    //if score > thresh tag ae as ce
    return score > thresh;

}

//bool vCornerManager::detectcorner(ev::vSurface2 *surf)
//{
//    double dx = 0.0, dy = 0.0, dxy = 0.0, score = 0.0;

////    int count = 0;

//    //get the most recent event
//    auto vr = is_event<AE>(surf->getMostRecent());

//    int l = windowRad - ((sobelsize - 1)/2);
//    int currxl = vr->x - l;
//    int currxh = vr->x + l;
//    int curryl = vr->y - l;
//    int curryh = vr->y + l;
//    for(int i = currxl; i <= currxh; i++) {
//        for(int j = curryl; j <= curryh; j++) {

//            //get the surface around the current event
//            const vQueue &subsurf = surf->getSurf(i, j, (sobelsize-1)/2);

//            int deltax = i - vr->x + l;
//            int deltay = j - vr->y + l;

////            std::cout << count << " " << vr->x << " " << i << " " << vr->y << " " << j << " "
////                      << deltax << " " << deltay << std::endl;
////            count++;

//            //compute the derivatives
//            double cx = convSobel(subsurf, sobelx, i, j);
//            double cy = convSobel(subsurf, sobely, i, j);
//            dx += gaussian(deltax, deltay) * pow(cx, 2);
//            dy += gaussian(deltax, deltay) * pow(cy, 2);
//            dxy += gaussian(deltax, deltay) * cx * cy;

//        }
//    }

//    //compute score
//    score = (dx*dy - dxy*dxy) - 0.04*((dx + dy) * (dx + dy));

//    if(debugPort.getOutputCount()) {
//        yarp::os::Bottle &scorebottleout = debugPort.prepare();
//        scorebottleout.clear();
//        scorebottleout.addDouble(score);
//        debugPort.write();
//    }

//    //if score > thresh tag ae as ce
//    return score > thresh;

//}
/**********************************************************/
double vCornerManager::convSobel(const ev::vQueue &w, yarp::sig::Matrix &sobel, int a, int b)
{
    double val = 0.0;

    //compute the convolution between the filter and the window
    for(unsigned int k = 0; k < w.size(); k++) {

        auto vi = is_event<AE>(w[k]);
        int deltax = vi->x - a + (sobelsize-1)/2;
        int deltay = vi->y - b + (sobelsize-1)/2;
//        std::cout << vi->x << " " << vi->y << " " << a << " " << b << " " << deltax << " " << deltay << " " << std::endl;
        val += sobel(deltax, deltay);
    }

//    for(int i = 0; i < sobelsize; i++) {
//        for(int j = 0; j < sobelsize; j++){
//            norm += fabs(sobel[i][j]);
//        }
//    }

    return val;

}

/**********************************************************/
//void vCornerManager::setSobelFilters(yarp::sig::Matrix &sobelx, yarp::sig::Matrix &sobely)
//{
//    yarp::sig::Vector Sx(sobelsize);
//    yarp::sig::Vector Dx(sobelsize);
//    for(int i = 1; i <= sobelsize; i++)
//    {
//        Sx(i - 1) = factorial((sobelsize - 1))/((factorial((sobelsize - 1)-(i - 1)))*(factorial(i - 1)));
//        Dx(i - 1) = Pasc(i - 1, sobelsize - 2)-Pasc(i - 2,sobelsize - 2);
//    }

//    sobelx = yarp::math::outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
//    sobely = sobelx.transposed();

//    double maxval = 0.0;
//    for(int k = 0; k < sobelsize; k++)
//    {
//        for(int j = 0; j < sobelsize; j++)
//        {
//            if((sobelx(k, j)) > maxval)
//                maxval = sobelx(k, j);
//        }
//    }

//    for(int k = 0; k < sobelsize; k++)
//    {
//        for(int j = 0; j < sobelsize; j++)
//        {
//            sobelx(k, j) = sobelx(k, j)/maxval;
//            sobely(k, j) = sobely(k, j)/maxval;
////            std::cout << sobelx(k, j) << " ";

//        }
////        std::cout << std::endl;
//    }

//}

//int vCornerManager::factorial(int a)
//{
//    if(a <= 1)
//        return 1;
//    return a*factorial(a - 1);
//}

//int vCornerManager::Pasc(int k, int n)
//{
//    int P;
//    if ((k >= 0) && (k <= n))
//        P = factorial(n)/(factorial(n-k)*factorial(k));
//    else
//        P = 0;
//    return P;
//}

/**********************************************************/
void vCornerManager::setGaussianFilter(double sigma, yarp::sig::Matrix &gaussian)
{
    double hsum = 0.0;
    const double A = 1.0/(2.0*M_PI*sigma*sigma);
    const int w = (gaussiansize-1)/2;
    for(int x = -w; x <= w; x++)
    {
        for(int y = -w; y <= w; y++)
        {
            const double hxy = A*exp(-(x*x + y*y)/(2*sigma*sigma));
            gaussian(w + x, w + y) = hxy;
            hsum += hxy;
        }
    }

    for(int x = -w; x <= w; x++)
    {
        for(int y = -w; y <= w; y++)
        {
            gaussian(w + x, w + y) = gaussian(w + x, w + y)/hsum;
//            std::cout << gaussian(w + x, w + y) << " ";
        }
//        std::cout << std::endl;
    }

}
//empty line to make gcc happy
