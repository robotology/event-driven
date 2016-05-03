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

#ifndef __VCORNER__
#define __VCORNER__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <math.h>

class vCornerManager : public yarp::os::BufferedPort<emorph::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<emorph::vBottle> outPort;

    emorph::vEdge *edge;

    int height;
    int width;
    int sobelsize;
    int thickness;
    double thresh;
    int fRad;
    unsigned int minEvts;

    yarp::sig::Matrix sobelx;
    yarp::sig::Matrix sobely;

    //for helping with timestamp wrap around
    emorph::vtsHelper unwrapper;

    bool detectcorner(); //(const emorph::vQueue &edge);
    double convSobel(const emorph::vQueue &subedge, yarp::sig::Matrix &sobel, int x, int y);
    inline double singleSobel(double val, yarp::sig::Matrix &sobel, int dx, int dy);
    int getMinStamp(const emorph::vQueue &subedge);
    int getMaxStamp(const emorph::vQueue &subedge);
    void setSobelFilters(int sobelsize, yarp::sig::Matrix &sobelx, yarp::sig::Matrix &sobely);
    int factorial(int a);
    int Pasc(int k, int n);

public:

    vCornerManager(int height, int width, int filterSize, int thickness, double thresh);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);

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
