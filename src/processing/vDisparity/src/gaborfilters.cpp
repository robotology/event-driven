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

gaborfilters::gaborfilters()
{

}

void gaborfilters::setCenter(int cx, int cy)
{
    this->cx = cx;
    this->cy = cy;
}

void gaborfilters::setParameters(double sigma, double orientation, double phase)
{
    this->sigma = sigma;
    this->fspatial = 1.0 / (2.0 * sigma);
    this->orientation = orientation;
    this->phase = phase;
}

double gaborfilters::process(emorph::vEvent &evt)
{
    emorph::AddressEvent *ae = evt.getAs<emorph::AddressEvent>();
    int dx = ae->getX() - cx;
    int dy = ae->getY() - cy;

    double dx_theta =  dx * cos(orientation) + dy * sin(orientation);
    double dy_theta = -dx * sin(orientation) + dy * cos(orientation);

    double gain = (1.0 / (2.0 * M_PI * pow(sigma, 2.0)));
    double gaussianComponent = exp( -( pow(dx_theta, 2.0) + pow(dy_theta, 2.0) ) / ( 2.0 * pow(sigma, 2.0) ) );
    double cosComponent = cos( (2.0 * M_PI * fspatial * dx_theta ) + phase );
    double response = gain * gaussianComponent * cosComponent;

    return response;

}

double gaborfilters::process(emorph::vQueue q)
{
    double response = 0;
    for(emorph::vQueue::iterator wi = q.begin(); wi != q.end(); wi++)
    {
        emorph::AddressEvent *vp = (*wi)->getAs<emorph::AddressEvent>();
        int dx = vp->getX() - cx;
        int dy = vp->getY() - cy;

        double dx_theta =  dx * cos(orientation) + dy * sin(orientation);
        double dy_theta = -dx * sin(orientation) + dy * cos(orientation);

        double gain = (1.0 / (2.0 * M_PI * pow(sigma, 2.0)));
        double gaussianComponent = exp( -( pow(dx_theta, 2.0) + pow(dy_theta, 2.0) ) / ( 2.0 * pow(sigma, 2.0) ) );
        double cosComponent = cos( (2.0 * M_PI * fspatial * dx_theta ) + phase );
        response += gain * gaussianComponent * cosComponent;

    }

    return response;

}

//empty line to make gcc happy
