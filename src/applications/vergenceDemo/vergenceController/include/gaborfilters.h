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

#ifndef __GABORFILTERS__
#define __GABORFILTERS__

#include <iCub/eventdriven/all.h>
#include <math.h>

class gaborfilter {

private:

    //parameters
    int cx;
    int cy;
    double orientation;
    double disppx;
    double sigma;
    double stdsperlambda;
    double fspatial;
    bool complexgabor;

    //precalculations
    double costheta;
    double sintheta;
    double neg2var;
    double coeff;

    double response;
    double evenresponse;
    double oddresponse;

public:

    gaborfilter();

    void setCenter(int cx, int cy);
    void setParameters(double sigma, double stdsperlambda, double orientation, double disppx);
    void setComplex(bool complex = true) { complexgabor = complex; }
    void process(ev::vEvent &evt, double gain = 1.0);
    void process(ev::vQueue &q, double gain = 1.0);
    double getResponse();
    void resetResponse();



};


#endif
//empty line to make gcc happy
