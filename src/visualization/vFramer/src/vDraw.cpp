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

    newDrawer = new lifeDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new clusterDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
	delete newDrawer;

    newDrawer = new integralDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new blobDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new circleDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new surfDraw();
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

    image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        cv::Vec3b cpc = image.at<cv::Vec3b>(aep->getX(), aep->getY());

        if(!aep->getPolarity())
        {
            //blue
            if(cpc[0] == 1) cpc[0] = 0;   //if positive and negative
            else cpc[0] = 160;            //if only positive
            //green
            if(cpc[1] == 60) cpc[1] = 255;
            else cpc[1] = 0;
            //red
            if(cpc[2] == 0) cpc[2] = 255;
            else cpc[2] = 160;
        }
        else
        {
            //blue
            if(cpc[0] == 160) cpc[0] = 0;   //negative and positive
            else cpc[0] = 1;                //negative only
            //green
            if(cpc[1] == 0) cpc[1] = 255;
            else cpc[1] = 60;
            //red
            if(cpc.val[2] == 160) cpc[2] = 255;
            else cpc[2] = 0;
        }

        image.at<cv::Vec3b>(aep->getX(), aep->getY()) = cpc;
    }
}

std::string lifeDraw::getTag()
{
    return "AEL";
}

void lifeDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    int cts = eSet.back()->getStamp();

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::FlowEvent *v = (*qi)->getAs<emorph::FlowEvent>();
        if(!v) continue;

        int modts = cts;
        if(cts < v->getStamp()) //we have wrapped
            modts += emorph::vtsHelper::maxStamp();

        if(modts > v->getDeath()) continue;

        cv::Vec3b cpc = image.at<cv::Vec3b>(v->getX(), v->getY());

        if(!v->getPolarity())
        {
            //blue
            if(cpc[0] == 1) cpc[0] = 0;   //if positive and negative
            else cpc[0] = 160;            //if only positive
            //green
            if(cpc[1] == 60) cpc[1] = 255;
            else cpc[1] = 0;
            //red
            if(cpc[2] == 0) cpc[2] = 255;
            else cpc[2] = 160;
        }
        else
        {
            //blue
            if(cpc[0] == 160) cpc[0] = 0;   //negative and positive
            else cpc[0] = 1;                //negative only
            //green
            if(cpc[1] == 0) cpc[1] = 255;
            else cpc[1] = 60;
            //red
            if(cpc.val[2] == 160) cpc[2] = 255;
            else cpc[2] = 0;
        }

        image.at<cv::Vec3b>(v->getX(), v->getY()) = cpc;
    }
}

std::string clusterDraw::getTag()
{
    return "CLE";
}

void clusterDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    std::stringstream ss;



    cv::Scalar red = CV_RGB(255, 0, 0);
    cv::Scalar green = CV_RGB(0, 255, 0);
    cv::Scalar color = red;

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(0);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        persistance.clear();
        return;
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
                    0, 0.3, CV_RGB(0, 0, 0));
    }

    cv::flip(textImg, textImg, 0);
    image += textImg;

}

std::string blobDraw::getTag()
{
    return "BLOB";
}

void blobDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    cv::Mat canvas(Xlimit, Ylimit, CV_8UC3);
    canvas.setTo(255);

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;

        cv::Vec3b cpc = canvas.at<cv::Vec3b>(aep->getX(), aep->getY());

        if(!aep->getPolarity())
        {
            cpc[0] = 0;
            cpc[1] = 0;
            cpc[2] = 0;
        }

        canvas.at<cv::Vec3b>(aep->getX(), aep->getY()) = cpc;

    }

    cv::medianBlur(canvas, canvas, 5);
    cv::blur(canvas, canvas, cv::Size(5, 5));
    cv::resize(canvas, image, cv::Size(Xlimit*2, Ylimit*2));
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

std::string circleDraw::getTag()
{
    return "CIRC";
}

void circleDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(255);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    //int n = 0;
    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::ClusterEventGauss *v = (*qi)->getAs<emorph::ClusterEventGauss>();
        if(!v) continue;
        cv::Point centr(v->getYCog(), v->getXCog());
        //we hide the radius in in X_sigma_2
        double r = v->getXSigma2();
        //we hide the radial variance in Y_sigma_2
        double w = v->getYSigma2();
        cv::circle(image, centr, r, CV_RGB(0, 0, 255), w);
        //n++;
    }

//    std::stringstream ss;
//    ss << n;
//    cv::putText(image, ss.str().c_str(), cv::Point(0, 0), 0, 0.5, CV_RGB(0, 0, 0), 1, 8, true);

}

std::string surfDraw::getTag()
{
    return "SURF";
}

void surfDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit*2, CV_8UC3);
    image.setTo(0);

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;

    double cts = eSet.back()->getStamp();

    emorph::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;

        //cpc[blue][green][red]

        if(v->getPolarity())
        {
            cv::Vec3b cpc = image.at<cv::Vec3b>(v->getX(), v->getY());
            if(cpc[1]) continue;

            double tts = v->getStamp();
            if(tts > cts) tts = tts - emorph::vtsHelper::maxStamp();
            if(cts - tts > gradient) continue;
            int val = 255 - 255.0 * (cts - tts)/ gradient;
            cpc[1] = std::max(val, 0);
            image.at<cv::Vec3b>(v->getX(), v->getY()) = cpc;

        }
        else
        {
            cv::Vec3b cpc = image.at<cv::Vec3b>(v->getX(), v->getY()+Ylimit);
            if(cpc[2]) continue;

            double tts = v->getStamp();
            if(tts > cts) tts = tts - emorph::vtsHelper::maxStamp();
            if(cts - tts > gradient) continue;
            int val = 255 - 255 * (cts - tts) / gradient;
            cpc[2] = std::max(val, 0);
            cpc[0] = std::max(val, 0);
            image.at<cv::Vec3b>(v->getX(), v->getY()+Ylimit) = cpc;
        }



    }

}

std::string flowDraw::getTag()
{
    return "FLOW";
}

void flowDraw::draw(cv::Mat &image, const emorph::vQueue &eSet)
{
    double k = 4;
    if(image.empty()) {
        image = cv::Mat(Xlimit*k, Ylimit*k, CV_8UC3);
        image.setTo(0);
    } else {
        cv::resize(image, image, cv::Size(0, 0), k, k, cv::INTER_LINEAR);
    }

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;


    int line_tickness = 1;
    cv::Scalar line_color = CV_RGB(255,0,0);
    cv::Point p_start,p_end;
    const double pi = 3.1416;

    int cts = eSet.back()->getAs<emorph::vEvent>()->getStamp();

    emorph::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        emorph::FlowEvent *ofp = (*qi)->getAs<emorph::FlowEvent>();
        if(!ofp) continue;

        int death = ofp->getDeath();
        if(cts < ofp->getStamp())
            death -= emorph::vtsHelper::maxStamp();
        if(cts > death || (death >= emorph::vtsHelper::maxStamp() &&
                           cts+emorph::vtsHelper::maxStamp() > death)) {
            continue;
        }

        int x = ofp->getY();
        int y = ofp->getX();
        float vx = ofp->getVx();
        float vy = ofp->getVy();

        //Starting point of the line
        p_start.x = x*k;
        p_start.y = y*k;

        double magnitude = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
        double hypotenuse = 0.5 / magnitude;
        if(hypotenuse < 5) hypotenuse = 5;
        if(hypotenuse > 20) hypotenuse = 20;
        hypotenuse = 15;
        double angle = atan2(vx, vy);

        //Scale the arrow by a factor of three
        p_end.x = (int) (p_start.x + hypotenuse * cos(angle));
        p_end.y = (int) (p_start.y + hypotenuse * sin(angle));

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 3*cos(angle + pi/4));
        p_start.y = (int) (p_end.y - 3*sin(angle + pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        p_start.x = (int) (p_end.x - 3*cos(angle - pi/4));
        p_start.y = (int) (p_end.y - 3*sin(angle - pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

    }

}

} //namesapce emorph

