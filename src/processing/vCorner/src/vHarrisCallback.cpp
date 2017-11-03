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

#include "vHarrisCallback.h"

using namespace ev;

vHarrisCallback::vHarrisCallback(int height, int width, double temporalsize, int qlen,
                                 int sobelsize, int windowRad, double sigma, double thresh)
{
    std::cout << "Using HARRIS implementation..." << std::endl;

    this->height = height;
    this->width = width;
    this->windowRad = windowRad;
    this->temporalsize = temporalsize / ev::vtsHelper::tsscaler;

    //ensure that sobel size is an odd number
    if(!(sobelsize % 2))
    {
        std::cout << "Warning: sobelsize should be odd" << std::endl;
        sobelsize--;
        std::cout << "sobelsize = " << sobelsize << " will be used" << std::endl;
    }

    this->qlen = qlen;
    this->thresh = thresh;

    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    //create surface representations
    std::cout << "Creating surfaces..." << std::endl;
    surfaceleft = new temporalSurface(width, height, this->temporalsize);
    surfaceright = new temporalSurface(width, height, this->temporalsize);

    this->tout = 0;

}
/**********************************************************/
bool vHarrisCallback::open(const std::string moduleName, bool strictness)
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
void vHarrisCallback::close()
{
    //close ports
    debugPort.close();
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    delete surfaceleft;
    delete surfaceright;

}

/**********************************************************/
void vHarrisCallback::interrupt()
{
    //pass on the interrupt call to everything needed
    debugPort.interrupt();
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}

/**********************************************************/
void vHarrisCallback::onRead(ev::vBottle &bot)
{
    ev::vBottle fillerbottle;
    bool isc = false;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto ae = is_event<AE>(*qi);
        ev::temporalSurface *cSurf;
        if(ae->getChannel() == 0)
            cSurf = surfaceleft;
        else
            cSurf = surfaceright;
        cSurf->fastAddEvent(*qi);

        vQueue subsurf;
        subsurf = cSurf->getSurf_Clim(qlen, ae->x, ae->y, windowRad);
        isc = detectcorner(subsurf, ae->x, ae->y);

        //if it's a corner, add it to the output bottle
        if(isc) {
            auto ce = make_event<LabelledAE>(ae);
            ce->ID = 1;
            fillerbottle.addEvent(ce);
        }

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
//            scorebottleout.addDouble(cpudelay);
            debugPort.write();
        }

    }

    if( (yarp::os::Time::now() - tout) > 0.001 && fillerbottle.size() ) {
        yarp::os::Stamp st;
        this->getEnvelope(st);
        outPort.setEnvelope(st);
        ev::vBottle &eventsout = outPort.prepare();
        eventsout.clear();
        eventsout = fillerbottle;
        outPort.write(strictness);
        fillerbottle.clear();
        tout = yarp::os::Time::now();
    }

}

/**********************************************************/
bool vHarrisCallback::detectcorner(const vQueue subsurf, int x, int y)
{

    //set the final response to be centred on the curren event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < subsurf.size(); i++)
    {
        //events are in the surface
        auto vi = is_event<AE>(subsurf[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

    //if score > thresh tag ae as ce
    return score > thresh;

}


//empty line to make gcc happy

