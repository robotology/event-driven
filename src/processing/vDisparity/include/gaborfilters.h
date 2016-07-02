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

class gaborfilters {

private:

    int cx;
    int cy;
    double orientation;
    double phase;
    double sigma;
    double fspatial;
    double evenresponse;
    double oddresponse;

public:

    gaborfilters();

    void setCenter(int cx, int cy);
    void setParameters(double sigma, double orientation, double phase);
    double process(emorph::vEvent &evt);
    double process(emorph::vQueue q);

};


#endif
//empty line to make gcc happy
