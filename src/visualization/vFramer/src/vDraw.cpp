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


    newDrawer = new flowDraw();
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

std::string flowDraw::getTag()
{
    return "OFE";
}

void flowDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    cv::Mat canvas(Xlimit, Ylimit, CV_8UC3);
    canvas.setTo(128);

    int line_tickness = 1;
    cv::Scalar line_color = CV_RGB(0,0,0);
    cv::Point p_start,p_end;
    int dx, dy;
    const double pi = 3.1416;

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::OpticalFlowEvent *ofp = (*qi)->getAs<emorph::OpticalFlowEvent>();
        if(ofp) {

            int x = ofp->getX();
            int y = ofp->getY();
            float vx = ofp->getVx();
            float vy = ofp->getVy();

            /*
            int X = 5+10*x;
            int Y = 5+10*y;

            if (vx<0 && vy<0)
            {
                dx = X+floor(vx-0.5);
                dy = Y+floor(vy-0.5);
            }

            if (vx>0 && vy<0)
            {
                dx = X+floor(vx+0.5);
                dy = Y+floor(vy-0.5);
            }

            if (vx<0 && vy>0)
            {
                dx = X+floor(vx-0.5);
                dy = Y+floor(vy+0.5);
            }

            if (vx>0 && vy>0)
            {
                dx = X+floor(vx+0.5);
                dy = Y+floor(vy+0.5);
            }

            */

            p_start.x = x;
            p_start.y = y;
            p_end.x = x + vx;
            p_end.y = y + vy;
            cv::line(canvas, p_start, p_end, line_color, line_tickness, CV_AA, 0);

            /*
            double angle;
            angle = atan2( (double) p_start.y - p_end.y, (double) p_start.x - p_end.x );

            p_start.x = (int) (p_end.x + 4 * cos(angle + pi/4));
            p_start.y = (int) (p_end.y + 4 * sin(angle + pi/4));
            cv::line(canvas, p_start, p_end, line_color, line_tickness, CV_AA, 0);

            p_start.x = (int) (p_end.x + 4 * cos(angle - pi/4));
            p_start.y = (int) (p_end.y + 4 * sin(angle - pi/4));
            cv::line(canvas, p_start, p_end, line_color, line_tickness, CV_AA, 0);
*/

      }

    }
    canvas.copyTo(image);

}

} //namesapce emorph

