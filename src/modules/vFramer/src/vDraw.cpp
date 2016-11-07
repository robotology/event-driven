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

#include "vDraw.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace eventdriven
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

    newDrawer = new fixedDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new fflowDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new isoDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new interestDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    return 0;
}


std::string addressDraw::getTag()
{
    return "AE";
}

void addressDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    if(eSet.empty()) return;

    eventdriven::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
        if(dt > twindow) break;


        eventdriven::AddressEvent *aep = (*qi)->getAs<eventdriven::AddressEvent>();
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

void lifeDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }
    if(eSet.empty()) return;

    int cts = eSet.back()->getStamp();

    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        eventdriven::FlowEvent *v = (*qi)->getAs<eventdriven::FlowEvent>();
        if(!v) continue;

        int modts = cts;
        if(cts < v->getStamp()) //we have wrapped
            modts += eventdriven::vtsHelper::maxStamp();

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

void clusterDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
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

    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        eventdriven::ClusterEvent *vp = (*qi)->getAs<eventdriven::ClusterEvent>();
        if(vp) {
            if(persistance[vp->getID()]) {
                delete persistance[vp->getID()];
                persistance[vp->getID()] = 0;
            }
            persistance[vp->getID()] = vp->clone()->getAs<eventdriven::ClusterEvent>();
            //latest[vp->getID()] = vp;
        }

    }

    std::map<int, eventdriven::ClusterEvent *>::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        if(!ci->second->getPolarity()) continue;

        cv::Point centr(ci->second->getYCog(), ci->second->getXCog());
        ss.str(""); ss << ci->second->getID();

        //the event could be a gaussian cluster or a regular cluster have
        //display options for both
        eventdriven::ClusterEventGauss *clegp = ci->second->getAs<eventdriven::ClusterEventGauss>();
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

void blobDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    cv::Mat canvas(Xlimit, Ylimit, CV_8UC3);
    canvas.setTo(255);

    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        eventdriven::AddressEvent *aep = (*qi)->getAs<eventdriven::AddressEvent>();
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

void integralDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    int d = 5;
    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        eventdriven::AddressEvent *aep = (*qi)->getAs<eventdriven::AddressEvent>();
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

void circleDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(255);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    //int n = 0;
    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
        if(dt > twindow) continue;

        eventdriven::ClusterEventGauss *v = (*qi)->getAs<eventdriven::ClusterEventGauss>();
        if(!v) continue;
        cv::Point centr(v->getYCog(), v->getXCog());
        //we hide the radius in in X_sigma_2
        double r = v->getXSigma2();
        //we hide the radial variance in Y_sigma_2
        double w = v->getYSigma2();

        CvScalar c;
        switch(v->getID()) {
        case(0): c = CV_RGB(0, 0, 255); break;
        case(1): c = CV_RGB(0, 255, 0); break;
        default: c = CV_RGB(255, 0, 0);
        }

        cv::circle(image, centr, r, c, w);
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

void surfDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit*2, CV_8UC3);
    image.setTo(0);

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;

    double cts = eSet.back()->getStamp();

    eventdriven::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        eventdriven::AddressEvent *v = (*qi)->getAs<eventdriven::AddressEvent>();
        if(!v) continue;

        //cpc[blue][green][red]

        if(v->getPolarity())
        {
            cv::Vec3b cpc = image.at<cv::Vec3b>(v->getX(), v->getY());
            if(cpc[1]) continue;

            double tts = v->getStamp();
            if(tts > cts) tts = tts - eventdriven::vtsHelper::maxStamp();
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
            if(tts > cts) tts = tts - eventdriven::vtsHelper::maxStamp();
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

void flowDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{
    double k = 4;
    if(image.empty()) {
        image = cv::Mat(Xlimit*k, Ylimit*k, CV_8UC3);
        image.setTo(255);
    } else {
        cv::resize(image, image, cv::Size(0, 0), k, k, cv::INTER_LINEAR);
    }

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;


    //double vx_mean = 0, vy_mean = 0, n = 0;
    int line_tickness = 1;
    cv::Scalar line_color = CV_RGB(255,0,0);
    cv::Point p_start,p_end;
    const double pi = 3.1416;

    int cts = eSet.back()->getAs<eventdriven::vEvent>()->getStamp();

    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
        if(dt > twindow) continue;

        eventdriven::FlowEvent *ofp = (*qi)->getAs<eventdriven::FlowEvent>();
        if(!ofp) continue;

//        int death = ofp->getDeath();
//        if(cts < ofp->getStamp())
//            death -= eventdriven::vtsHelper::maxStamp();
//        if(cts > death || (death >= eventdriven::vtsHelper::maxStamp() &&
//                           cts+eventdriven::vtsHelper::maxStamp() > death)) {
//            continue;
//        }

        int modts = cts;
        if(cts < ofp->getStamp()) //we have wrapped
            modts += eventdriven::vtsHelper::maxStamp();
        if(modts > ofp->getDeath()) continue;

        int x = ofp->getY();
        int y = ofp->getX();
        float vx = ofp->getVy();
        float vy = ofp->getVx();
        //vx_mean += vx;
        //vy_mean += vy;
        //n++;

        //Starting point of the line
        p_start.x = x*k;
        p_start.y = y*k;

        double magnitude = sqrt(pow(vx, 2.0f) + pow(vy, 2.0f));
        double hypotenuse = magnitude;
        if(hypotenuse < 10) hypotenuse = 10;
        //if(hypotenuse > 20) hypotenuse = 20;
        hypotenuse = 20;
        double angle = atan2(vy, vx);

        //Scale the arrow by a factor of three
        p_end.x = (int) (p_start.x + hypotenuse * sin(angle));
        p_end.y = (int) (p_start.y + hypotenuse * cos(angle));
        //p_end.x = (int) (p_start.x + ofp->getVx() * 2);
        //p_end.y = (int) (p_start.y + ofp->getVy() * 2);

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 7*sin(angle + pi/4));
        p_start.y = (int) (p_end.y - 7*cos(angle + pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        p_start.x = (int) (p_end.x - 7*sin(angle - pi/4));
        p_start.y = (int) (p_end.y - 7*cos(angle - pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

    }
    //std::cout << "y: " << vy_mean << "x: " << vx_mean << std::endl;
    //cv::resize(image, image, cv::Size(0, 0), 1.0/k, 1.0/k, cv::INTER_LINEAR);
}

std::string fixedDraw::getTag()
{
    return "FIXED";
}

void fixedDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    int i = 0;
    eventdriven::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        eventdriven::AddressEvent *aep = (*qi)->getAs<eventdriven::AddressEvent>();
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

        if(++i > 2000) return;
    }
}

std::string fflowDraw::getTag()
{
    return "FFLOW";
}

void fflowDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{
    double k = 4;
    if(image.empty()) {
        image = cv::Mat(Xlimit*k, Ylimit*k, CV_8UC3);
        image.setTo(255);
    } else {
        cv::resize(image, image, cv::Size(0, 0), k, k, cv::INTER_LINEAR);
    }

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;


    //double vx_mean = 0, vy_mean = 0, n = 0;
    int line_tickness = 1;
    cv::Scalar line_color = CV_RGB(255,0,0);
    cv::Point p_start,p_end;
    const double pi = 3.1416;

    //int cts = eSet.back()->getAs<eventdriven::vEvent>()->getStamp();

    int i = 0;
    eventdriven::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        if(++i > 2000) return;
        eventdriven::FlowEvent *ofp = (*qi)->getAs<eventdriven::FlowEvent>();
        if(!ofp) continue;

//        int death = ofp->getDeath();
//        if(cts < ofp->getStamp())
//            death -= eventdriven::vtsHelper::maxStamp();
//        if(cts > death || (death >= eventdriven::vtsHelper::maxStamp() &&
//                           cts+eventdriven::vtsHelper::maxStamp() > death)) {
//            continue;
//        }

        //int modts = cts;
        //if(cts < ofp->getStamp()) //we have wrapped
        //    modts += eventdriven::vtsHelper::maxStamp();
        //if(modts > ofp->getDeath()) continue;

        int x = ofp->getY();
        int y = ofp->getX();
        float vx = ofp->getVy();
        float vy = ofp->getVx();
        //vx_mean += vx;
        //vy_mean += vy;
        //n++;

        //Starting point of the line
        p_start.x = x*k;
        p_start.y = y*k;

        double magnitude = sqrt(pow(vx, 2.0f) + pow(vy, 2.0f));
        double hypotenuse = magnitude;
        if(hypotenuse < 10) hypotenuse = 10;
        //if(hypotenuse > 20) hypotenuse = 20;
        //hypotenuse = 40;
        double angle = atan2(vy, vx);

        //Scale the arrow by a factor of three
//        p_end.x = (int) (p_start.x + hypotenuse * cos(angle));
//        p_end.y = (int) (p_start.y + hypotenuse * sin(angle));
        p_end.x = (int) (p_start.x + ofp->getVx() * 2);
        p_end.y = (int) (p_start.y + ofp->getVy() * 2);

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 7*sin(angle + pi/4));
        p_start.y = (int) (p_end.y - 7*cos(angle + pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        p_start.x = (int) (p_end.x - 7*sin(angle - pi/4));
        p_start.y = (int) (p_end.y - 7*cos(angle - pi/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

    }
    //std::cout << "y: " << vy_mean << "x: " << vx_mean << std::endl;

}

void isoDraw::initialise()
{
    maxdt = 1;

    theta1 = -40 * 3.14 / 180.0;
    theta2 = 20 * 3.14 / 180.0;

    c1 = cos(theta1); s1 = sin(theta1);
    c2 = cos(theta2); s2 = sin(theta2);

    //int x = c1*px - s1*pts + imagexshift;
    //int y = c2*py - s2*(-s1*px - c1*pts) + imageyshift;
    scale = Xlimit * 3;
    imagexshift = 0 + 10;
    imageyshift = -(c2*0 - s2*(-s1*Xlimit - c1*0)) + 1;

    int imageymax = c2*Ylimit - s2*(-s1*0 - c1*scale);
    int imagexmax = c1*Xlimit - s1*scale;

    imagewidth = imagexmax + imagexshift + 1; //(c1 * Xlimit - s1 * 0) - (c1 * 0 - s1 * Ylimit) + 2;
    imageheight = imageymax + imageyshift + 1; // * 2;


    baseimage = cv::Mat(imageheight, imagewidth, CV_8UC3);
    baseimage.setTo(0);


    //cv::putText(baseimage, std::string("X"), cv::Point(100, 100), 1, 0.5, CV_RGB(0, 0, 0));

    cv::Scalar c = CV_RGB(125, 125, 125);
    int x, y;
    for(int xi = 0; xi < Xlimit; xi++) {
        x = c1*xi - s1*0 + imagexshift;
        y = c2*0 - s2*(-s1*xi - c1*0) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        if(xi == Xlimit / 2) {
            cv::putText(baseimage, std::string("x"), cv::Point(x-10, y-9),
                        cv::FONT_ITALIC, 0.5, c, 1, 8, true);
        }

        x = c1*xi - s1*0 + imagexshift;
        y = c2*Ylimit - s2*(-s1*xi - c1*0) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        //x = c1*xi - s1*scale + imagexshift;
        //y = c2*Ylimit - s2*(-s1*xi - c1*scale) + imageyshift;
        //baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

    }

    for(int yi = 0; yi <= Ylimit; yi++) {
        x = c1*0 - s1*0 + imagexshift;
        y = c2*yi - s2*(-s1*0 - c1*0) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        if(yi == Ylimit / 2) {
            cv::putText(baseimage, std::string("y"), cv::Point(x-9, y),
                        cv::FONT_ITALIC, 0.5, c, 1, 8, true);
        }

        x = c1*Xlimit - s1*0 + imagexshift;
        y = c2*yi - s2*(-s1*Xlimit - c1*0) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        //x = c1*Xlimit - s1*scale + imagexshift;
        //y = c2*yi - s2*(-s1*Xlimit - c1*scale) + imageyshift;
        //baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

    }

    int tsi;
    for(tsi = 0; tsi < (int)(scale*0.3); tsi++) {
        x = c1*Xlimit - s1*tsi + imagexshift;
        y = c2*0 - s2*(-s1*Xlimit - c1*tsi) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        if(tsi == (int)(scale *0.15)) {
            cv::putText(baseimage, std::string("t"), cv::Point(x, y-12),
                        cv::FONT_ITALIC, 0.5, c, 1, 8, true);
        }

        //x = c1*Xlimit - s1*tsi + imagexshift;
        //y = c2*Ylimit - s2*(-s1*Xlimit - c1*tsi) + imageyshift;
        //baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        //x = c1*0 - s1*tsi + imagexshift;
        //y = c2*Ylimit - s2*(-s1*0 - c1*tsi) + imageyshift;
        //baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
    }

    for(int i = 0; i < 14; i++) {
        x = c1*(Xlimit-i/2) - s1*(tsi-i) + imagexshift;
        y = c2*0 - s2*(-s1*(Xlimit-i/2) - c1*(tsi-i)) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

        x = c1*(Xlimit+i/2) - s1*(tsi-i) + imagexshift;
        y = c2*0 - s2*(-s1*(Xlimit+i/2) - c1*(tsi-i)) + imageyshift;
        baseimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);

    }

}

std::string isoDraw::getTag()
{
    return "ISO";
}

void isoDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    if(eSet.empty()) return;

    int cts = eSet.back()->getStamp();
    int dt = cts - eSet.front()->getStamp();
    if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
    maxdt = std::max(maxdt, dt);


    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        eventdriven::AddressEvent *aep = (*qi)->getAs<eventdriven::AddressEvent>();
        if(!aep) continue;

        //transform values
        double dt = cts - aep->getStamp();
        if(dt < 0) dt += eventdriven::vtsHelper::maxStamp();
        dt /= (double)maxdt;
        int pts = dt * scale;
        int px = aep->getY();
        int py = aep->getX();

        int x = c1*px - s1*pts + imagexshift;
        int y = c2*py - s2*(-s1*px - c1*pts) + imageyshift;

        //int x = c1 * ts - s1 * (Ylimit - aep->getY()) + imagexshift;
        //int y = c2 * (s1 * ts + c1 * (Ylimit - aep->getY())) - (s2 * aep->getX());
//        int x = c1 * aep->getX() - s1 * (Ylimit - aep->getY()) + imagexshift;
//        int y = c2 * (s1 * aep->getX()+ c1 * (Ylimit - aep->getY())) - (s2 * ts);

        if(x < 0 || x >= imagewidth || y < 0 || y >= imageheight) {
            continue;
        }

        if(!aep->getPolarity()) {
//            if(dt < 0.9)
//                isoimage.at<cv::Vec3b>(y, x) = cv::Vec3b(80, 0, 80);
//            else
                isoimage.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 160, 255);
        } else {
//            if(dt < 0.9)
//                isoimage.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 80, 0);
//            else
                isoimage.at<cv::Vec3b>(y, x) = cv::Vec3b(160, 255, 160);
        }

    }

    if(!image.empty()) {
        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {

                if(image.at<cv::Vec3b>(y, x)[0] != 255
                        || image.at<cv::Vec3b>(y, x)[1] != 255
                        || image.at<cv::Vec3b>(y, x)[2] != 255) {

                    int tx = c1*x - s1*0 + imagexshift;
                    int ty = c2*y - s2*(-s1*x - c1*0) + imageyshift;
                    if(tx < 0 || tx >= imagewidth || ty < 0 || ty >= imageheight)
                        continue;

                    isoimage.at<cv::Vec3b>(ty, tx) = image.at<cv::Vec3b>(y, x);
                }
            }
        }
    }



    image = isoimage - baseimage;

}

std::string interestDraw::getTag()
{
    return "AE-INT";
}

void interestDraw::draw(cv::Mat &image, const eventdriven::vQueue &eSet)
{

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(255);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    int r = 1;
    CvScalar c = CV_RGB(255, 0, 0);
    eventdriven::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        eventdriven::InterestEvent *v = (*qi)->getAs<eventdriven::InterestEvent>();
        if(!v) continue;
        cv::Point centr(v->getY(), v->getX());
        cv::circle(image, centr, r, c);
    }

}

} //namesapce eventdriven

