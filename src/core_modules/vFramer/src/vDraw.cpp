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

using ev::event;
using ev::vQueue;
using ev::getas;

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

void addressDraw::draw(cv::Mat &image, const ev::vQueue &eSet)
{
    image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;


        event<ev::AddressEvent> aep = getas<ev::AddressEvent>(*qi);
        if(!aep) continue;

        cv::Vec3b &cpc = image.at<cv::Vec3b>(aep->getY(), aep->getX());

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
    }
}

std::string lifeDraw::getTag()
{
    return "AEL";
}

void lifeDraw::draw(cv::Mat &image, const vQueue &eSet)
{

    image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }
    if(eSet.empty()) return;

    int cts = eSet.back()->getStamp();

    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        event<ev::FlowEvent> v = getas<ev::FlowEvent>(*qi);
        if(!v) continue;

        int modts = cts;
        if(cts < v->getStamp()) //we have wrapped
            modts += ev::vtsHelper::maxStamp();

        if(modts > v->getDeath()) continue;

        cv::Vec3b &cpc = image.at<cv::Vec3b>(v->getY(), v->getX());

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
    }
}

std::string clusterDraw::getTag()
{
    return "CLE";
}

void clusterDraw::draw(cv::Mat &image, const vQueue &eSet)
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

    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        event<ev::ClusterEvent> vp = getas<ev::ClusterEvent>(*qi);
        if(vp) {
            persistance[vp->getID()] = vp;
        }

    }

    std::map<int, event<ev::ClusterEvent> >::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        if(!ci->second->getPolarity()) continue;

        cv::Point centr(ci->second->getYCog(), ci->second->getXCog());
        ss.str(""); ss << ci->second->getID();

        //the event could be a gaussian cluster or a regular cluster have
        //display options for both
        event<ev::ClusterEventGauss> clegp = getas<ev::ClusterEventGauss>(ci->second);
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

void blobDraw::draw(cv::Mat &image, const ev::vQueue &eSet)
{

    cv::Mat canvas(Ylimit, Xlimit, CV_8UC3);
    canvas.setTo(255);

    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        event<ev::AddressEvent> aep = getas<ev::AddressEvent>(*qi);
        if(!aep) continue;

        cv::Vec3b &cpc = canvas.at<cv::Vec3b>(aep->getY(), aep->getX());

        if(!aep->getPolarity())
        {
            cpc[0] = 0;
            cpc[1] = 0;
            cpc[2] = 0;
        }
    }

    cv::medianBlur(canvas, canvas, 5);
    cv::blur(canvas, canvas, cv::Size(5, 5));
    cv::resize(canvas, image, cv::Size(Ylimit, Xlimit));
}

integralDraw::integralDraw()
{
    iimage = cv::Mat(Ylimit, Xlimit, CV_8UC1);
    iimage.setTo(128);
}

std::string integralDraw::getTag()
{
    return "INTI";
}

void integralDraw::draw(cv::Mat &image, const vQueue &eSet)
{

    int d = 5;
    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        event<ev::AddressEvent> aep = getas<ev::AddressEvent>(*qi);
        if(aep) {

            unsigned char c = iimage.at<unsigned char>(aep->getY(), aep->getX());
            int i = c;
            if(aep->getPolarity())
                i += d;
            else
                i -= d;

            //i =+ aep->getPolarity()?d:-d;
            i = std::max(i, 0);
            i = std::min(i, 255);
            iimage.at<unsigned char>(aep->getY(), aep->getX()) = (unsigned char)i;
        }

    }

    cv::cvtColor(iimage, image, CV_GRAY2BGR);
}

std::string circleDraw::getTag()
{
    return "CIRC";
}

void circleDraw::draw(cv::Mat &image, const vQueue &eSet)
{

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(255);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) continue;

        event<ev::ClusterEventGauss> v = getas<ev::ClusterEventGauss>(*qi);
        if(!v) continue;
        cv::Point centr(v->getXCog(), v->getYCog());
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
    }
}

std::string surfDraw::getTag()
{
    return "SURF";
}

void surfDraw::draw(cv::Mat &image, const vQueue &eSet)
{

    image = cv::Mat(Ylimit, Xlimit*2, CV_8UC3);
    image.setTo(0);

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;

    double cts = eSet.back()->getStamp();

    vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        event<ev::AddressEvent> v = getas<ev::AddressEvent>(*qi);
        if(!v) continue;

        //cpc[blue][green][red]

        if(v->getPolarity())
        {
            cv::Vec3b cpc = image.at<cv::Vec3b>(v->getY(), v->getX());
            if(cpc[1]) continue;

            double tts = v->getStamp();
            if(tts > cts) tts = tts - ev::vtsHelper::maxStamp();
            if(cts - tts > gradient) continue;
            int val = 255 - 255.0 * (cts - tts)/ gradient;
            cpc[1] = std::max(val, 0);
            image.at<cv::Vec3b>(v->getY(), v->getX()) = cpc;

        }
        else
        {
            cv::Vec3b cpc = image.at<cv::Vec3b>(v->getY(), v->getX()+Ylimit);
            if(cpc[2]) continue;

            double tts = v->getStamp();
            if(tts > cts) tts = tts - ev::vtsHelper::maxStamp();
            if(cts - tts > gradient) continue;
            int val = 255 - 255 * (cts - tts) / gradient;
            cpc[2] = std::max(val, 0);
            cpc[0] = std::max(val, 0);
            image.at<cv::Vec3b>(v->getY(), v->getX()+Ylimit) = cpc;
        }



    }

}

std::string flowDraw::getTag()
{
    return "FLOW";
}

void flowDraw::draw(cv::Mat &image, const vQueue &eSet)
{
    double k = 2;
    if(image.empty()) {
        image = cv::Mat(Ylimit*k, Xlimit*k, CV_8UC3);
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

    //int cts = eSet.back()->getAs<eventdriven::vEvent>()->getStamp();

    vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->getStamp() - (*qi)->getStamp();
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;

        event<ev::FlowEvent> ofp = getas<ev::FlowEvent>(*qi);
        if(!ofp) continue;

//        int death = ofp->getDeath();
//        if(cts < ofp->getStamp())
//            death -= eventdriven::vtsHelper::maxStamp();
//        if(cts > death || (death >= eventdriven::vtsHelper::maxStamp() &&
//                           cts+eventdriven::vtsHelper::maxStamp() > death)) {
//            continue;
//        }

//        int modts = cts;
//        if(cts < ofp->getStamp()) //we have wrapped
//            modts += eventdriven::vtsHelper::maxStamp();
//        if(modts > ofp->getDeath()) continue;

        int x = ofp->getX();
        int y = ofp->getY();
        float vx = ofp->getVx();
        float vy = ofp->getVy();
        //vx_mean += vx;
        //vy_mean += vy;
        //n++;

        //Starting point of the line
        p_start.x = x*k;
        p_start.y = y*k;

        double magnitude = sqrt(pow(vx, 2.0f) + pow(vy, 2.0f));
        double hypotenuse = magnitude;
        //if(hypotenuse < 10) hypotenuse = 10;
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
        p_start.x = (int) (p_end.x - 5*sin(angle + M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle + M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        p_start.x = (int) (p_end.x - 5*sin(angle - M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle - M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

    }
    //std::cout << "y: " << vy_mean << "x: " << vx_mean << std::endl;
    //cv::resize(image, image, cv::Size(0, 0), 1.0/k, 1.0/k, cv::INTER_LINEAR);
}

std::string fixedDraw::getTag()
{
    return "FIXED";
}

void fixedDraw::draw(cv::Mat &image, const vQueue &eSet)
{

    image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    int i = 0;
    vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        event<ev::AddressEvent> aep = getas<ev::AddressEvent>(*qi);
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

void fflowDraw::draw(cv::Mat &image, const vQueue &eSet)
{
    double k = 4;
    if(image.empty()) {
        image = cv::Mat(Ylimit*k, Xlimit*k, CV_8UC3);
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

    //int cts = eSet.back()->getAs<eventdriven::vEvent>()->getStamp();

    int i = 0;
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        if(++i > 2000) return;
        event<ev::FlowEvent> ofp = getas<ev::FlowEvent>(*qi);
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

        int x = ofp->getX();
        int y = ofp->getY();
        float vx = ofp->getVx();
        float vy = ofp->getVy();
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
        p_start.x = (int) (p_end.x - 7*sin(angle + M_PI/4));
        p_start.y = (int) (p_end.y - 7*cos(angle + M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

        p_start.x = (int) (p_end.x - 7*sin(angle - M_PI/4));
        p_start.y = (int) (p_end.y - 7*cos(angle - M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_tickness, CV_AA);

    }
    //std::cout << "y: " << vy_mean << "x: " << vx_mean << std::endl;

}

void isoDraw::pttr(int &x, int &y, int &z) {
    // we want a negative rotation around the y axis (yaw)
    // a positive rotation around the x axis (pitch) (no roll)
    // the z should always be negative values.
    // the points need to be shifted across by negligble amount
    // the points need to be shifted up by (x = max, y = 0, ts = 0 rotation)

    int xmod = x*CY + z*SY + 0.5; // +0.5 rounds rather than floor
    int ymod = y*CX - SX*(-x*SY + z*CY) + 0.5;
    int zmod = y*SX + CX*(-x*SY + z*CY) + 0.5;
    x = xmod; y = ymod; z = zmod;
}

void isoDraw::initialise()
{
    maxdt = 1; //just to initialise (but we don't use here)
    Zlimit = Xlimit * 3;

    thetaX = 20 * 3.14 / 180.0;  //PITCH
    thetaY = 40 * 3.14 / 180.0; //YAW

    CY = cos(thetaY); SY = sin(thetaY);
    CX = cos(thetaX); SX = sin(thetaX);

    //the following calculations make the assumption of a negative yaw and
    //a positive pitch
    int x, y, z;
    int maxx = 0, maxy = 0, miny = Ylimit, minx = Xlimit;
    for(int xi = 0; xi <= Xlimit; xi+=Xlimit) {
        for(int yi = 0; yi <= Ylimit; yi+=Ylimit) {
            for(int zi = 0; zi <= Zlimit; zi+=Zlimit) {
                x = xi; y = yi; z = zi; pttr(x, y, z);
                maxx = std::max(maxx, x);
                maxy = std::max(maxy, y);
                minx = std::min(minx, x);
                miny = std::min(miny, y);
            }
        }
    }


    imagexshift = -minx + 10;
    imageyshift = -miny + 10;

    imagewidth = maxx + imagexshift + 10;
    imageheight = maxy + imageyshift + 10;

    baseimage = cv::Mat(imageheight, imagewidth, CV_8UC3);
    baseimage.setTo(0);


    //cv::putText(baseimage, std::string("X"), cv::Point(100, 100), 1, 0.5, CV_RGB(0, 0, 0));

    cv::Scalar invertedtextc = CV_RGB(125, 125, 125);
    cv::Vec3b invertedaxisc = cv::Vec3b(255, 255, 255);

    for(int xi = 0; xi < Xlimit; xi++) {
        x = xi; y = 0; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        x = xi; y = Ylimit; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(xi == Xlimit / 2) {
            cv::putText(baseimage, std::string("x"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
    }

    for(int yi = 0; yi <= Ylimit; yi++) {
        x = 0; y = yi; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
        if(yi == Ylimit / 2) {
            cv::putText(baseimage, std::string("y"), cv::Point(x-10, y+10),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }
        x = Xlimit; y = yi; z = 0; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

    }

    int tsi;
    for(tsi = 0; tsi < (int)(Zlimit*0.3); tsi++) {

        x = Xlimit; y = Ylimit; z = tsi; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

        if(tsi == (int)(Zlimit *0.15)) {
            cv::putText(baseimage, std::string("t"), cv::Point(x, y+12),
                        cv::FONT_ITALIC, 0.5, invertedtextc, 1, 8, false);
        }

    }

    for(int i = 0; i < 14; i++) {

        x = Xlimit-i/2; y = Ylimit; z = tsi-i; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;

        x = Xlimit+i/2; y = Ylimit; z = tsi-i; pttr(x, y, z);
        y += imageyshift; x += imagexshift;
        baseimage.at<cv::Vec3b>(y, x) = invertedaxisc;
    }

}

std::string isoDraw::getTag()
{
    return "ISO";
}

void isoDraw::draw(cv::Mat &image, const ev::vQueue &eSet)
{

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    if(eSet.empty()) return;

    int cts = eSet.back()->getStamp();
    int dt = cts - eSet.front()->getStamp();
    if(dt < 0) dt += ev::vtsHelper::maxStamp();
    maxdt = std::max(maxdt, dt);

    ev::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        event<ev::AddressEvent> aep = getas<ev::AddressEvent>(*qi);
        if(!aep) continue;

        //transform values
        int dt = cts - aep->getStamp();
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        dt = ((double)dt / maxdt) * Zlimit + 0.5;
        int px = aep->getX();
        int py = aep->getY();
        int pz = dt;
        pttr(px, py, pz);
        px += imagexshift;
        py += imageyshift;

        if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight) {
            continue;
        }

        if(!aep->getPolarity()) {
                isoimage.at<cv::Vec3b>(py, px) = cv::Vec3b(255, 160, 255);
        } else {
                isoimage.at<cv::Vec3b>(py, px) = cv::Vec3b(160, 255, 160);
        }

    }

    if(!image.empty()) {
        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {
                cv::Vec3b &pixel = image.at<cv::Vec3b>(y, x);

                if(pixel[0] != 255 || pixel[1] != 255 || pixel[2] != 255) {

                    int px = x, py = y, pz = 0; pttr(px, py, pz);
                    px += imagexshift;
                    py += imageyshift;
                    if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight)
                        continue;

                    isoimage.at<cv::Vec3b>(py, px) = pixel;
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

void interestDraw::draw(cv::Mat &image, const ev::vQueue &eSet)
{

    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    int r = 1;
    CvScalar c = CV_RGB(255, 0, 0);
    ev::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        event<ev::InterestEvent> v = getas<ev::InterestEvent>(*qi);
        if(!v) continue;
        cv::Point centr(v->getY(), v->getX());
        cv::circle(image, centr, r, c);
    }

}


