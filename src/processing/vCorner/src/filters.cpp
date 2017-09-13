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

#include "filters.h"

using namespace ev;

void filters::configure(int sobelsize, int l)
{
    cx = 0;
    cy = 0;
    this->sobelsize = sobelsize;
    sobelrad = (sobelsize - 1)/2;

    rx = 0;
    ry = 0;
    this->l = l;
    lrad = (l - 1)/2;

    dx = 0.0;
    dy = 0.0;
    dxy = 0.0;
    sobelx.resize(sobelsize, sobelsize);
    sobely.resize(sobelsize, sobelsize);
    gaussian.resize(l, l);
    responsex.resize(l, l);
    responsey.resize(l, l);

}

void filters::setFilterCenter(int cx, int cy)
{
    this->cx = cx;
    this->cy = cy;
}

void filters::setResponseCenter(int rx, int ry)
{
    this->rx = rx;
    this->ry = ry;
}

void filters::reset() {

    responsex.zero();
    responsey.zero();
    dx = 0.0;
    dy = 0.0;
    dxy = 0.0;
}

void filters::applysobel(ev::event<AE> evt)
{

    //apply sobel filters
    int lx = std::max(evt->x-sobelrad, rx-lrad);
    int ux = std::min(evt->x+sobelrad, rx+lrad);
    int ly = std::max(evt->y-sobelrad, ry-lrad);
    int uy = std::min(evt->y+sobelrad, ry+lrad);
    for(int cx = lx; cx <= ux; cx++)
    {
        for(int cy = ly; cy <= uy; cy++)
        {
            //(cx,cy) is the pixel where we apply the sobel filter
            this->setFilterCenter(cx, cy);

            int diffx = evt->x - cx;
            int diffy = evt->y - cy;

            double gainx = sobelx(diffx + sobelrad, diffy + sobelrad);
            double gainy = sobely(diffx + sobelrad, diffy + sobelrad);
            this->responsex(cx - rx + lrad, cy - ry + lrad) += gainx;
            this->responsey(cx - rx + lrad, cy - ry + lrad) += gainy;

        }
    }

}

void filters::applygaussian()
{
    for(int m = 0; m < l; m++)
    {
        for(int n = 0; n < l; n++)
        {
            double sx = this->responsex(m, n);
            double sy = this->responsey(m, n);
            double g = gaussian(m, n);
            dx += g * pow(sx, 2);
            dy += g * pow(sy, 2);
            dxy += g * sx * sy;
        }
    }
}

double filters::getScore()
{
    return (dx*dy - dxy*dxy) - 0.04*((dx + dy) * (dx + dy));
}

void filters::setSobelFilters()
{
    yarp::sig::Vector Sx(sobelsize);
    yarp::sig::Vector Dx(sobelsize);
    for(int i = 1; i <= sobelsize; i++)
    {
        Sx(i - 1) = factorial((sobelsize - 1))/((factorial((sobelsize - 1)-(i - 1)))*(factorial(i - 1)));
        Dx(i - 1) = Pasc(i - 1, sobelsize - 2)-Pasc(i - 2,sobelsize - 2);
    }

    this->sobelx = yarp::math::outerProduct(Sx, Dx); //Mx=Sy(:)*Dx;
    this->sobely = sobelx.transposed();

    double maxval = 0.0;
    for(int k = 0; k < sobelsize; k++)
    {
        for(int j = 0; j < sobelsize; j++)
        {
            if((sobelx(k, j)) > maxval)
                maxval = sobelx(k, j);
        }
    }

    for(int k = 0; k < sobelsize; k++)
    {
        for(int j = 0; j < sobelsize; j++)
        {
            sobelx(k, j) = sobelx(k, j)/maxval;
            sobely(k, j) = sobely(k, j)/maxval;
        }
    }
}

int filters::factorial(int a)
{
    if(a <= 1)
        return 1;
    return a*factorial(a - 1);
}

int filters::Pasc(int k, int n)
{
    int P;
    if ((k >= 0) && (k <= n))
        P = factorial(n)/(factorial(n-k)*factorial(k));
    else
        P = 0;
    return P;
}

void filters::setGaussianFilter(double sigma)
{
    double hsum = 0.0;
    const double A = 1.0/(2.0*M_PI*sigma*sigma);
    const int w = (l-1)/2;
    for(int x = -w; x <= w; x++)
    {
        for(int y = -w; y <= w; y++)
        {
            const double hxy = A*exp(-(x*x + y*y)/(2*sigma*sigma));
            this->gaussian(w + x, w + y) = hxy;
            hsum += hxy;
        }
    }

    for(int x = -w; x <= w; x++)
    {
        for(int y = -w; y <= w; y++)
        {
            gaussian(w + x, w + y) = gaussian(w + x, w + y)/hsum;
        }
    }

}


//empty line to make gcc happy
