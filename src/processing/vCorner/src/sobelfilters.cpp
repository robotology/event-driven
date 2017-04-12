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

#include "sobelfilters.h"

using namespace ev;

sobelfilter::sobelfilter()
{
    cx = 0;
    cy = 0;
    sobelsize = 5;
    sobelrad = (sobelsize - 1)/2;
    l = 7;
    lrad = (l - 1)/2;
    sobelx.resize(sobelsize, sobelsize);
    sobely.resize(sobelsize, sobelsize);
    responsex.resize(l, l);
    responsey.resize(l, l);

}

void sobelfilter::setCenter(int cx, int cy)
{
    this->cx = cx;
    this->cy = cy;
}

double sobelfilter::getResponseX(int i, int j) {

    return responsex(i, j);
}

double sobelfilter::getResponseY(int i, int j) {

    return responsey(i, j);
}

void sobelfilter::resetResponses() {

    responsex.zero();
    responsey.zero();
}

//std::pair<double, double> sobelfilter::process(ev::event<AE> evt)
//{
//    //auto ae = is_event<AE>(evt.clone());
//    int dx = evt->x - cx + sobelrad;
//    int dy = evt->y - cy + sobelrad;
//    return std::make_pair (sobelx(dx, dy), sobely(dx, dy));
////    return gain;
//}

//void sobelfilter::updateresponse(vEvent &curr_evt, vEvent &cent_evt)
//{
//    auto cente = is_event<AE>(cent_evt.clone());
//    auto curre = is_event<AE>(curr_evt.clone());
//    int centerx = cx - cente->x + lrad;
//    int centery = cy - cente->y + lrad;

//    this->responsex(centerx, centery) += process(*curre).first;
//    this->responsey(centerx, centery) += process(*curre).second;

//}

void sobelfilter::process(ev::event<AE> evt, int currx, int curry)
{
    int dx = evt->x - cx;
    int dy = evt->y - cy;
    int centerx = cx - currx + lrad;
    int centery = cy - curry + lrad;

    double gainx = sobelx(dx + sobelrad, dy + sobelrad);
    double gainy = sobely(dx + sobelrad, dy + sobelrad);

    this->responsex(centerx, centery) += gainx;
    this->responsey(centerx, centery) += gainy;

}

void sobelfilter::setSobelFilters(int sobelsize)
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

int sobelfilter::factorial(int a)
{
    if(a <= 1)
        return 1;
    return a*factorial(a - 1);
}

int sobelfilter::Pasc(int k, int n)
{
    int P;
    if ((k >= 0) && (k <= n))
        P = factorial(n)/(factorial(n-k)*factorial(k));
    else
        P = 0;
    return P;
}

void sobelfilter::printFilters()
{
    for(int i = 0; i < sobelsize; i++)
    {
        for(int j = 0; j < sobelsize; j++)
        {
            std::cout << sobelx(i, j) << " ";
        }
    std::cout << std::endl;
    }

    std::cout << std::endl;
    for(int i = 0; i < sobelsize; i++)
    {
        for(int j = 0; j < sobelsize; j++)
        {
            std::cout << sobely(i, j) << " ";
        }
    std::cout << std::endl;
    }
}

//empty line to make gcc happy
