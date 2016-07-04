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

#include <iCub/emorph/all.h>
#include <math.h>

class gaborfilter {

private:

    //parameters
    int cx;
    int cy;
    double orientation;
    double phase;
    double sigma;
    double fspatial;

    //precalculations
    double costheta;
    double sintheta;
    double neg2var;
    double coscoeff;

    double response;
    //double evenresponse;
    //double oddresponse;

public:

    gaborfilter();

    void setCenter(int cx, int cy);
    void setParameters(double sigma, double orientation, double phase);
    void process(emorph::vEvent &evt, double gain = 1.0);
    void process(emorph::vQueue &q, double gain = 1.0);
    double getResponse() { return response; }
    void resetResponse() { response = 0.0; }


};


#endif
//empty line to make gcc happy
