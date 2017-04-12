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

    if(tag == addressDraw::drawtype)
        return new addressDraw();
    if(tag == isoDraw::drawtype)
        return new isoDraw();
    if(tag == interestDraw::drawtype)
        return new interestDraw();
    if(tag == flowDraw::drawtype)
        return new flowDraw();
    if(tag == lifeDraw::drawtype)
        return new lifeDraw();
    if(tag == clusterDraw::drawtype)
        return new clusterDraw();
    if(tag == blobDraw::drawtype)
        return new blobDraw();
    return 0;

}

const std::string addressDraw::drawtype = "AE";

std::string addressDraw::getDrawType()
{
    return addressDraw::drawtype;
}

std::string addressDraw::getEventType()
{
    return AddressEvent::tag;
}

void addressDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{
    image = cv::Mat(Ylimit, Xlimit, CV_8UC3);
    image.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
 //       return;
    }

    if(eSet.empty()) return;

    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {


        int dt = eSet.back()->stamp - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) break;


        auto aep = is_event<AddressEvent>(*qi);
        int y = aep->y;
        int x = aep->x;
        if(flip) {
            y = Ylimit - 1 - y;
            x = Xlimit - 1 - x;
        }


        cv::Vec3b &cpc = image.at<cv::Vec3b>(y, x);

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

const std::string lifeDraw::drawtype = "LIFE";

std::string lifeDraw::getDrawType()
{
    return lifeDraw::drawtype;
}

std::string lifeDraw::getEventType()
{
    return FlowEvent::tag;
}

void lifeDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
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

const std::string clusterDraw::drawtype = "CLE";

std::string clusterDraw::getDrawType()
{
    return clusterDraw::drawtype;
}

std::string clusterDraw::getEventType()
{
    return GaussianAE::tag;
}

void clusterDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
{
    cv::Scalar blue = CV_RGB(0, 0, 255);

    if(image.empty()) {
        image = cv::Mat(Xlimit, Ylimit, CV_8UC3);
        image.setTo(0);
    }

    if(checkStagnancy(eSet) > clearThreshold) {
        //        persistance.clear();
        //        return;
    }

    if(eSet.empty()) return;

    //update the 'persistence' the current state of each of the cluster ID's
    for(vQueue::const_iterator qi = eSet.begin(); qi != eSet.end(); qi++) {
        auto vp = is_event<GaussianAE>(*qi);
        if(vp) {
            persistance[vp->ID] = vp;
        }
    }

    std::map<int, event<GaussianAE> >::iterator ci;
    for(ci = persistance.begin(); ci != persistance.end(); ci++) {

        auto v = ci->second;

        //polarity indicates the cluster has died.
        if(!v->polarity) continue;

        cv::Point centr(v->x, v->y);
        if(flip) {
            centr.x = Xlimit - 1 - centr.x;
            centr.y = Ylimit - 1 - centr.y;
        }

        double sig_x2_ = v->sigx;
        double sig_y2_ = v->sigy;
        double sig_xy_ = v->sigxy;
        double tmp = sqrt( (sig_x2_ - sig_y2_) * (sig_x2_ - sig_y2_) + 4*sig_xy_*sig_xy_ );
        double l_max = 0.5*(sig_x2_ + sig_y2_ + tmp);
        double l_min = 0.5*(sig_x2_ + sig_y2_ - tmp);

        if(l_min < -5) {
            std::cout << "l_min error: shape distorted" << std::endl;
        }

        double a = sqrt(std::fabs(l_max)) * 5;
        double b = sqrt(std::fabs(l_min)) * 5;
        double alpha = 0.5*atan2f(2*sig_xy_, sig_y2_ - sig_x2_);

        alpha = alpha * 180 / M_PI; //convert to degrees for openCV ellipse function
        cv::ellipse(image, centr, cv::Size(a,b), alpha, 0, 360, blue, 2);
        //cv::circle(image, centr, v->sigx, blue, 2);
    }

}

const std::string blobDraw::drawtype = "BLOB";

std::string blobDraw::getDrawType()
{
    return blobDraw::drawtype;
}

std::string blobDraw::getEventType()
{
    return AE::tag;
}

void blobDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
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

const std::string flowDraw::drawtype = "FLOW";

std::string flowDraw::getDrawType()
{
    return flowDraw::drawtype;
}

std::string flowDraw::getEventType()
{
    return FlowEvent::tag;
}

void flowDraw::draw(cv::Mat &image, const vQueue &eSet, int vTime)
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

        auto ofp = is_event<ev::FlowEvent>(*qi);

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

const std::string interestDraw::drawtype = "AE-INT";

std::string interestDraw::getDrawType()
{
    return interestDraw::drawtype;
}

std::string interestDraw::getEventType()
{
    return LabelledAE::tag;
}

void interestDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
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
    ev::vQueue::const_reverse_iterator qi;
    for(qi = eSet.rbegin(); qi != eSet.rend(); qi++) {
        int dt = eSet.back()->stamp - (*qi)->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        if(dt > twindow) continue;

        auto v = as_event<ev::LabelledAE>(*qi);
        if(!v) continue;
        cv::Point centr(v->x, v->y);
        cv::circle(image, centr, r, c);
    }

}

const std::string isoDraw::drawtype = "ISO";

std::string isoDraw::getDrawType()
{
    return isoDraw::drawtype;
}

std::string isoDraw::getEventType()
{
    return AddressEvent::tag;
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

void isoDraw::draw(cv::Mat &image, const ev::vQueue &eSet, int vTime)
{

    cv::Mat isoimage = baseimage.clone();
    isoimage.setTo(255);

    if(checkStagnancy(eSet) > clearThreshold) {
       // return;
    }

    if(eSet.empty()) return;

    int cts = eSet.back()->stamp;
    int dt = cts - eSet.front()->stamp;
    if(dt < 0) dt += ev::vtsHelper::maxStamp();
    maxdt = std::max(maxdt, dt);

    ev::vQueue::const_iterator qi;
    for(qi = eSet.begin(); qi != eSet.end(); qi++) {

        auto aep = is_event<AE>(*qi);

        //transform values
        int dt = cts - aep->stamp;
        if(dt < 0) dt += ev::vtsHelper::maxStamp();
        dt = ((double)dt / maxdt) * Zlimit + 0.5;
        int px = aep->x;
        int py = aep->y;
        if(flip) {
            px = Xlimit - 1 - px;
            py = Ylimit - 1 - py;
        }
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




