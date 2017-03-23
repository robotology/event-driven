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

using namespace ev;

vDraw * createDrawer(std::string tag)
{
    vDraw * newDrawer = 0;

    newDrawer = new addressDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new flowDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new isoDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new interestDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new lifeDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new clusterDraw();
    if(tag == newDrawer->getTag()) return newDrawer;
    delete newDrawer;

    newDrawer = new blobDraw();
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


        int dt = eSet.back()->stamp - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;


        auto aep = as_event<AddressEvent>(*qi);
        if(!aep) continue;

        cv::Vec3b &cpc = image.at<cv::Vec3b>(aep->y, aep->x);

        if(!aep->polarity)
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

    int cts = eSet.back()->stamp;

    vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        auto v = as_event<FlowEvent>(*qi);
        if(!v) continue;

        int modts = cts;
        if(cts < v->stamp) //we have wrapped
            modts += ev::vtsHelper::maxStamp();

        if(modts > v->getDeath()) continue;

        cv::Vec3b &cpc = image.at<cv::Vec3b>(v->y, v->x);

        if(!v->polarity)
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
    cv::Scalar blue = CV_RGB(0, 0, 255);
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
        auto vp = as_event<LabelledAE>(*qi);
        if(vp) {
            persistance[vp->ID] = vp;
        }

    }

    std::map<int, event<LabelledAE> >::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        if(!ci->second->polarity) continue;

        cv::Point centr(ci->second->y, ci->second->x);
        ss.str(""); ss << ci->second->ID;

        //the event could be a gaussian cluster or a regular cluster have
        //display options for both
        auto clegp = as_event<GaussianAE>(ci->second);
        if(clegp) {
            double sig_x2_ = clegp->sigx;
            double sig_y2_ = clegp->sigy;
            double sig_xy_ = clegp->sigxy;
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

            switch(clegp->ID) {
            case(0): color = blue; break;
            case(1): color = green; break;
            default: color = red;
            }

            cv::ellipse(image, centr, cv::Size(a,b), alpha, 0, 360, color);

        } else {
            switch(ci->second->ID) {
            case(0): color = blue; break;
            case(1): color = green; break;
            default: color = red;
            }
            cv::circle(image, centr, 4, color);
        }

        cv::putText(textImg, ss.str(),
                    cv::Point(ci->second->y,
                              Xlimit - ci->second->x),
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

    image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
        return;
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;

        auto aep = as_event<AE>(*qi);
        if(!aep) continue;
        if(!aep->polarity)
            image.at<cv::Vec3b>(aep->y, aep->x) = cv::Vec3b(0, 0, 0);
    }

    cv::medianBlur(image, image, 5);
    cv::blur(image, image, cv::Size(5, 5));
}

//std::string circleDraw::getTag()
//{
//    return "CIRC";
//}

//void circleDraw::draw(cv::Mat &image, const vQueue &eSet)
//{

//    if(image.empty()) {
//        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
//        image.setTo(255);
//    }

//    if(checkStagnancy(eSet) > clearThreshold) {
//        return;
//    }

//    vQueue::const_iterator qi;
//    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

//        int dt = eSet.back()->stamp - (*qi)->stamp;
//        if(dt < 0) dt += ev::vtsHelper::maxStamp();
//        if(dt > twindow) continue;

//        event<ev::ClusterEventGauss> v = as_event<ev::ClusterEventGauss>(*qi);
//        if(!v) continue;
//        cv::Point centr(v->getXCog(), v->getYCog());
//        //we hide the radius in in X_sigma_2
//        double r = v->getXSigma2();
//        //we hide the radial variance in Y_sigma_2
//        double w = v->getYSigma2();

//        CvScalar c;
//        switch(v->getID()) {
//            case(0): c = CV_RGB(0, 0, 255); break;
//            case(1): c = CV_RGB(0, 255, 0); break;
//            default: c = CV_RGB(255, 0, 0);
//        }

//        cv::circle(image, centr, r, c, w);
//    }
//}


std::string flowDraw::getTag()
{
    return "FLOW";
}

void flowDraw::draw(cv::Mat &image, const vQueue &eSet)
{
    if(image.empty()) {
        image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
        image.setTo(255);
    }

    if(eSet.empty()) return;
    if(checkStagnancy(eSet) > clearThreshold) return;


    //double vx_mean = 0, vy_mean = 0, n = 0;
    int line_thickness = 0;
    cv::Scalar line_color = CV_RGB(255,0,0);
    cv::Point p_start,p_end;

    vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::max_stamp;
        if(dt > twindow/4) break;

        auto ofp = as_event<ev::FlowEvent>(*qi);
        if(!ofp) continue;

        int x = ofp->x;
        int y = ofp->y;
        float vx = ofp->vx;
        float vy = ofp->vy;

        //Starting point of the line
        p_start.x = x;
        p_start.y = y;

        double hypotenuse = 15;
        double angle = atan2(vy, vx);

        //Scale the arrow by a factor of three
        p_end.x = (int) (p_start.x + hypotenuse * sin(angle));
        p_end.y = (int) (p_start.y + hypotenuse * cos(angle));

        //Draw the main line of the arrow
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        //Draw the tips of the arrow
        p_start.x = (int) (p_end.x - 5*sin(angle + M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle + M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

        p_start.x = (int) (p_end.x - 5*sin(angle - M_PI/4));
        p_start.y = (int) (p_end.y - 5*cos(angle - M_PI/4));
        cv::line(image, p_start, p_end, line_color, line_thickness, 4);

    }

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
        auto v = as_event<ev::LabelledAE>(*qi);
        if(!v) continue;
        cv::Point centr(v->y, v->x);
        cv::circle(image, centr, r, c);
    }

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

    int cts = eSet.back()->stamp;
    int dt = cts - eSet.front()->stamp;
    if(dt < 0) dt += ev::vtsHelper::maxStamp();
    maxdt = std::max(maxdt, dt);

    ev::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {
        event<ev::AddressEvent> aep = as_event<ev::AddressEvent>(*qi);
        if(!aep) continue;

        //transform values
        int dt = cts - aep->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        dt = ((double)dt / maxdt) * Zlimit + 0.5;
        int px = aep->x;
        int py = aep->y;
        int pz = dt;
        pttr(px, py, pz);
        px += imagexshift;
        py += imageyshift;

        if(px < 0 || px >= imagewidth || py < 0 || py >= imageheight) {
            continue;
        }

        if(!aep->polarity) {
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




