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

#include "gaborfilters.h"

using namespace ev;

gaborfilter::gaborfilter()
{
    cx = 0;
    cy = 0;

    orientation = 0;
    disppx = 0;
    sigma = 0;
    stdsperlambda = 6.0;
    fspatial = 1.0 / (2.0 * sigma);

    costheta = cos(orientation);
    sintheta = sin(orientation);
    neg2var = -2.0 * pow(sigma, 2.0);
    coeff = 2.0 * M_PI * fspatial;

    response = 0;
    evenresponse = 0;
    oddresponse = 0;
    complexgabor = true;
}

void gaborfilter::setCenter(int cx, int cy)
{
    this->cx = cx;
    this->cy = cy;
}

void gaborfilter::setParameters(double sigma, double stdsperlambda, double orientation, double disppx)
{
    this->sigma = sigma;
    this->stdsperlambda = stdsperlambda;
    this->fspatial = 1.0 / (this->stdsperlambda * sigma);
    this->orientation = orientation;
    this->disppx = disppx;

    costheta = cos(orientation);
    sintheta = sin(orientation);
    neg2var = -2.0 * pow(sigma, 2.0);
    coeff = 2.0 * M_PI * fspatial;
}

double gaborfilter::getResponse() {
    if(!complexgabor)
        return response;
    else
        return sqrt(pow(evenresponse, 2.0) + pow(oddresponse, 2.0));
}

void gaborfilter::resetResponse() {
    response = 0.0;
    oddresponse = 0.0;
    evenresponse = 0.0;
}

void gaborfilter::process(ev::vEvent &evt, double gain)
{
    auto ae = as_event<AE>(evt.clone());

    if(!ae) return;
    int dx = ae->x - cx;
    int dy = ae->y - cy;

    double dx_theta =  dx * costheta + dy * sintheta;
    double dy_theta = -dx * sintheta + dy * costheta;

    //gain is extra computation that should be the same for all filters
    //double gain = (1.0 / (2.0 * M_PI * pow(sigma, 2.0)));

    //the gaussian component assumes a circular Gaussian shape? should it be an elipse instead?
    //double gaussianComponent = exp( (pow(dx_theta, 2.0) + pow(dy_theta, 2.0)) / neg2var );
    double gaussianComponent = exp( /*(pow(dx_theta, 2.0) +*/ pow(dy_theta, 2.0) / neg2var );
    //add in the even component also
    double cosComponent = 0.0;
    double sinComponent = 0.0;
    if(ae->getChannel())
    {
        cosComponent = cos( coeff * (dx_theta + disppx ) );
        sinComponent = sin( coeff * (dx_theta + disppx ) );
    }
    else
    {
        cosComponent = cos( (coeff * dx_theta ) );
        sinComponent = sin( (coeff * dx_theta ) );
    }


    if(complexgabor)
    {
        evenresponse += gain * gaussianComponent * cosComponent;
        oddresponse  += gain * gaussianComponent * sinComponent;
        //response = evenresponse * evenresponse + oddresponse * oddresponse;
    }
    else
    {
        response += gain * gaussianComponent * cosComponent;
    }
}

void gaborfilter::process(ev::vQueue &q, double gain)
{
    for(ev::vQueue::iterator wi = q.begin(); wi != q.end(); wi++)
        process(**wi, gain);
}

//empty line to make gcc happy
