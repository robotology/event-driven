/*
 * Copyright (C) 2010 eMorph Group iCub Facility
 * Authors: Arren Glover
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

#include "iCub/vDraw.h"

namespace emorph
{

vDraw * createDrawer(std::string tag)
{
    vDraw * newDrawer = 0;

    newDrawer = new addressDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;


    newDrawer = new integralDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    return 0;
}


std::string addressDraw::getTag()
{
    return "AE";
}

void addressDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    cv::Mat canvas(Xlimit, Ylimit, CV_8UC3);
    canvas.setTo(128);

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(aep) {
            cv::Vec3b cpc = canvas.at<cv::Vec3b>(aep->getX(), aep->getY());

            if(aep->getPolarity())
            {
                if(cpc[0]) cpc[0] = 255;
                if(cpc[1]) cpc[1] = 255;
                cpc[2] = 255;
            }
            else
            {
                cpc[0] = 0;
                cpc[1] = 0;
                if(cpc.val[2] < 255) cpc[2] = 0;
            }

            canvas.at<cv::Vec3b>(aep->getX(), aep->getY()) = cpc;
        }

    }

    canvas.copyTo(image);

}

integralDraw::integralDraw()
{
    iimage = cv::Mat(Xlimit, Ylimit, CV_8UC1);
    iimage.setTo(128);
}

std::string integralDraw::getTag()
{
    return "INTI";
}

void integralDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    int d = 5;
    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(aep) {

            unsigned char c = iimage.at<unsigned char>(aep->getX(), aep->getY());
            int i = c;
            if(aep->getPolarity())
                i += d;
            else
                i -= d;

            //i =+ aep->getPolarity()?d:-d;
            i = std::max(i, 0);
            i = std::min(i, 255);
            iimage.at<unsigned char>(aep->getX(), aep->getY()) = (unsigned char)i;
        }

    }

    cv::cvtColor(iimage, image, CV_GRAY2BGR);
    //iimage.copyTo(image);

}

} //namesapce emorph

