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

#ifndef __SOBELFILTERS__
#define __SOBELFILTERS__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <utility>

class sobelfilter {

private:

    //parameters
    int cx;
    int cy;
    int sobelsize;
    int sobelrad;
    int l;
    int lrad;

    yarp::sig::Matrix sobelx;
    yarp::sig::Matrix sobely;
    yarp::sig::Matrix responsex;
    yarp::sig::Matrix responsey;

public:

    sobelfilter();

    void setCenter(int cx, int cy);
    void setSobelFilters(int sobelsize);
    int factorial(int a);
    int Pasc(int k, int n);
    std::pair<double, double> process(ev::event<ev::AE> evt);
    void updateresponse(ev::vEvent &curr_evt, ev::vEvent &cent_evt);
    void process(ev::event<ev::AE> evt, int currx, int curry);
    double getResponseX(int i, int j);
    double getResponseY(int i, int j);
    void resetResponses();
    void printFilters();

};


#endif
//empty line to make gcc happy
