/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
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

#ifndef __ICUB_SALIENTPOS__
#define __ICUB_SALIENTPOS__

#include <yarp/os/all.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <algorithm>


class YARPsalientI : public yarp::os::BufferedPort<emorph::vBottle>
{
private:
    
    //for helping with timestamp wrap around
    emorph::vtsHelper unwrapper;

    // output vBottle
    yarp::os::BufferedPort<emorph::vBottle> vBottleOut;

    std::vector<float>* xList;
    std::vector<float>* yList;
    int fullHeight;
    int fullWidth;
    int prevTS;
    int thresh;
    int maxXCount;
    int maxX;
    int maxYCount;
    int maxY;
    float decay;
    int clusterRadiusX;
    int clusterRadiusY;



public:
    
    YARPsalientI();


    bool    open(const std::string &name);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);


};


class vSalientPos : public yarp::os::RFModule
{

    //the event bottle input and output handler

    YARPsalientI      inputManager;


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
