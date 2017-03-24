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

/// \defgroup Modules Modules
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VCORNER__
#define __VCORNER__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <sobelfilters.h>
#include <fstream>
#include <math.h>

class vCornerManager : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //parameters
    int height;
    int width;
    int sobelsize;
    int sobelrad;
    int windowRad;
    double sigma;
    int nEvents;
    double thresh;

    int gaussiansize;

//    yarp::sig::Matrix sobelx;
//    yarp::sig::Matrix sobely;
    sobelfilter sobel;
    yarp::sig::Matrix gaussian;

    //data structures
    ev::vSurface2 *surfaceOnL;
    ev::vSurface2 *surfaceOfL;
    ev::vSurface2 *surfaceOnR;
    ev::vSurface2 *surfaceOfR;

    //for helping with timestamp wrap around
    ev::vtsHelper unwrapper;

    bool detectcorner(ev::vSurface2 *surf);
    double convSobel(const ev::vQueue &window, yarp::sig::Matrix &sobel, int a, int b);
//    void setSobelFilters(yarp::sig::Matrix &sobelx, yarp::sig::Matrix &sobely);
//    int factorial(int a);
//    int Pasc(int k, int n);
    void setGaussianFilter(double sigma, yarp::sig::Matrix &gaussian);

public:

    vCornerManager(int height, int width, int filterSize, int windowRad, double sigma, int nEvents, double thresh);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);

};

class vCornerModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vCornerManager      *cornermanager;


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};


#endif
//empty line to make gcc happy
