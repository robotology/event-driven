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

    newDrawer = new clusterDraw();
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

std::string clusterDraw::getTag()
{
    return "CLE";
}

void clusterDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    std::stringstream ss;
    //std::map<int, emorph::ClusterEvent *> latest;

    cv::Scalar red = CV_RGB(255, 0, 0);
    cv::Scalar green = CV_RGB(0, 255, 0);
    cv::Scalar color = red;

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(0);
    }


    cv::Mat textImg(image.rows, image.cols, image.type()); textImg.setTo(0);

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::ClusterEvent *vp = (*qi)->getAs<emorph::ClusterEvent>();
        if(vp) {
            if(persistance[vp->getID()]) {
                delete persistance[vp->getID()];
                persistance[vp->getID()] = 0;
            }
            persistance[vp->getID()] = vp->clone()->getAs<emorph::ClusterEvent>();
            //latest[vp->getID()] = vp;
        }

    }

    std::map<int, emorph::ClusterEvent *>::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        if(!ci->second->getPolarity()) continue;

        cv::Point centr(ci->second->getYCog(), ci->second->getXCog());
        ss.str(""); ss << ci->second->getID();

        //the event could be a gaussian cluster or a regular cluster have
        //display options for both
        emorph::ClusterEventGauss *clegp = ci->second->getAs<emorph::ClusterEventGauss>();
        if(clegp) {
            double sig_x2_ = clegp->getXSigma2();
            double sig_y2_ = clegp->getYSigma2();
            double sig_xy_ = clegp->getXYSigma();
            double tmp = sqrt( (sig_x2_ - sig_y2_) * (sig_x2_ - sig_y2_) + 4*sig_xy_*sig_xy_ );
            double l_max = 0.5*(sig_x2_ + sig_y2_ + tmp);
            double l_min = 0.5*(sig_x2_ + sig_y2_ - tmp);


            if(l_min < -5) {
                std::cout << "l_min error: shape distorted" << std::endl;
            }

            double a = sqrt(std::fabs(l_max)) * 2;
            double b = sqrt(std::fabs(l_min)) * 2;
            double alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);

            alpha = alpha * 180 / M_PI; //convert to degrees for openCV ellipse function

            if(clegp->getPolarity())
                color = green;
            else
                color = red;

            cv::ellipse(image, centr, cv::Size(a,b), alpha, 0, 360, color);

        } else {
            cv::circle(image, centr, 4, green);
        }

        cv::putText(textImg, ss.str(),
                    cv::Point(ci->second->getYCog(),
                              Xlimit - ci->second->getXCog()),
                    0, 0.3, CV_RGB(255, 255, 255));
    }

    cv::flip(textImg, textImg, 0);
    image += textImg;

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

