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

#ifndef __FILTERS__
#define __FILTERS__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <utility>

class filters {

private:

    //parameters for sobel filters
    int cx;
    int cy;
    int sobelsize;
    int sobelrad;

    //parameters of the final response
    int rx;
    int ry;
    int l;
    int lrad;

    //result
    double dx;
    double dy;
    double dxy;

    //matrices to store filters and responses
    yarp::sig::Matrix sobelx;
    yarp::sig::Matrix sobely;
    yarp::sig::Matrix gaussian;
    yarp::sig::Matrix responsex;
    yarp::sig::Matrix responsey;

public:

    filters() {}
    void configure(int sobelsize, int l);
    void setFilterCenter(int cx, int cy);
    void setResponseCenter(int rx, int ry);
    void setSobelFilters();
    void setGaussianFilter(double sigma);
    int factorial(int a);
    int Pasc(int k, int n);
    void applysobel(ev::event<ev::AE> evt);
    void applygaussian();
    double getScore();
    void reset();

};


#endif
//empty line to make gcc happy
