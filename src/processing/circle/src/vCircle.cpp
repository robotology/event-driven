/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#include "vCircle.h"
#include <vector>
//#include <opencv2/opencv.hpp>

//void vCircle::createLocalSearch(int x, int y)
//{
//    localActivity.clear();
//    int tminx = std::max(x-sRadius, 0);
//    int tmaxx = std::min(x+sRadius, width -1);
//    int tminy = std::max(y-sRadius, 0);
//    int tmaxy = std::min(y+sRadius, height-1);

//    for(int u = tminx; u <= tmaxx; u++) {
//        localActivity.push_back(act_unit(u, tminy, activity.queryActivity(u, tminy)));
//        localActivity.push_back(act_unit(u, tmaxy, activity.queryActivity(u, tmaxy)));
//    }
//    for(int v = tminy+1; v <= tmaxy-1; v++) {
//        localActivity.push_back(act_unit(tminx, v, activity.queryActivity(tminx, v)));
//        localActivity.push_back(act_unit(tmaxx, v, activity.queryActivity(tmaxx, v)));
//    }

//}

void vCircle::createLocalSearch(int x, int y)
{
    localActivity.clear();
    int tminx = std::max(x-sRadius, 0);
    int tmaxx = std::min(x+sRadius, width -1);
    int tminy = std::max(y-sRadius, 0);
    int tmaxy = std::min(y+sRadius, height-1);
    for(int v = tminy; v <= tmaxy; v++) {
        for(int u = tminx; u <= tmaxx; u++) {
            double a = activity.queryActivity(u, v);
            if(a > 1)
                localActivity.push_back(act_unit(u, v, a));
        }
    }
}

cv::Mat vCircle::createLocalActivityWindow(int x, int y)
{
    cv::Mat submat(sRadius*2+1, sRadius*2+1, CV_64F); submat.setTo(0.8);
    int tminx = std::max(x-sRadius, 0);
    int tmaxx = std::min(x+sRadius, width -1);
    int tminy = std::max(y-sRadius, 0);
    int tmaxy = std::min(y+sRadius, height-1);

    //std::cout << "[0-" << sRadius+1 << "]" << std::endl;
    for(int v = tminy; v <= tmaxy; v++) {
        for(int u = tminx; u <= tmaxx; u++) {
            //std::cout << v - (y-sRadius) << " " <<  u - (x-sRadius) << ". ";
            double a = activity.queryActivity(u, v);
            submat.at<double>(v - (y-sRadius), u - (x-sRadius)) = a;
        }
    }
    std::cout << std::endl;
    return submat;
}

void vCircle::pointTrim(int x, int y)
{
    std::vector<act_unit>::iterator i = localActivity.begin();
    while(i != localActivity.end()) {
        if(sqrt(std::pow(x - i->x, 2.0) + std::pow(y - i->y, 2)) < tRadius)
            localActivity.erase(i);
        else
            i++;
    }
}

void vCircle::linearTrim(int x1, int y1, int x2, int y2)
{
    std::vector<act_unit>::iterator i = localActivity.begin();
    if(x2 == x1) {
        //if we have a vertical line just check x value
        while(i != localActivity.end()) {
            if(std::fabs(i->x - x1) < tRadius)
                localActivity.erase(i);
            else
                i++;
        }
    } else {
        //otherwise we have to calculate the line parameters
        double m = (y2 - y1) / (double)(x2 - x1);
        double b = y1 - m * x1;
        while(i != localActivity.end()) {
            if(std::fabs(i->y - m*i->x - b) < tRadius)
                localActivity.erase(i);
            else
                i++;
        }
    }
}


double vCircle::calculateCircleActivity(int cx, int cy, int r)
{
    if(r == 0) return 0;

    double actacc = 0;
    int px = cx; int py = cy;
    int x = px; int y = py;
    double a = 0;
    while(a < 6.18) {
        while(x == px && y == py) {
            a += 0.05;
            x = r * std::sin(a);
            y = r * std::cos(a);
        }
        px = x; py = y;
        if(x >= 0 && y >= 0 && x < width && y < height)
            actacc += activity.queryActivity(x, y);
    }
    return actacc;
}

bool vCircle::localCircleEstimate(emorph::AddressEvent &event, double &cx,
                                  double &cy, double &cr)
{
    act_unit p1(event.getX(), event.getY(), 0);
    //update the activity here
    p1.a = activity.addEvent(event);

    //create our search locations
    createLocalSearch(p1.x, p1.y);

    //remove points too close to centre
    pointTrim(p1.x, p1.y);
    if(localActivity.size() < 3) return false;

    //find the best score
    int bi = 0;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > localActivity[bi].a) bi = i;
    act_unit p2 = localActivity[bi];

    //remove activity close-by the first point
    linearTrim(p1.x, p1.y, p2.x, p2.y);
    if(localActivity.empty()) return false;

    //find the second best score
    bi = 0;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > localActivity[bi].a) bi = i;
    act_unit p3 = localActivity[bi];

    //if we are all on the same y line (should be impossible after linearTrim)
    if(p1.y == p2.y && p1.y == p3.y) return false;

    //make sure x2 is different to x1 and x3 (else we divide by 0 later)
    act_unit * ex = 0;

    if(p2.x == p1.x) {
        if(p2.x == p3.x)
            return 0;
        ex = &p3;
    } else if(p2.x == p3.x) {
        ex = &p1;
    }

    if(ex) {
        act_unit temp = p2;
        p2 = *ex;
        *ex = temp;
    }


    // /////////////////////////////////////////////////////////////////////////
    //%%% display activity in the activity window

//    cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
//    for(int x = 0; x < 128; x++) {
//        for(int y = 0; y < 128; y++) {
//            image2.at<float>(x, y) = activity.queryActivity(x, y);
//        }
//    }
//    cv::Mat image(512, 512, CV_32F);
//    cv::resize(image2, image, image.size());

//    cv::circle(image, cv::Point(p1.y, p1.x)*4, 12, CV_RGB(255, 255, 255));
//    cv::circle(image, cv::Point(p2.y, p2.x)*4, 12, CV_RGB(255, 255, 255));
//    cv::circle(image, cv::Point(p3.y, p3.x)*4, 12, CV_RGB(255, 255, 255));
    //cv::imshow("Local Activity", image);
    //cv::waitKey(1);

    // /////////////////////////////////////////////////////////////////////////

    //calculate the circle from the 3 points
    double ma = (p2.y - p1.y) / (double)(p2.x - p1.x);
    double mb = (p3.y - p2.y) / (double)(p3.x - p2.x);

    cx = (ma * mb * (p1.y - p3.y) + mb * (p1.x + p2.x) -
                        ma * (p2.x + p3.x)) / (2 * (mb - ma));
    if(ma)
        cy = -1 * (cx - (p1.x+p2.x)/2.0)/ma + (p1.y+p2.y)/2.0;
    else
        cy = -1 * (cx - (p2.x+p3.x)/2.0)/mb + (p2.y+p3.y)/2.0;

    cr = sqrt(pow(cx - p1.x, 2.0) + pow(cy - p1.y, 2.0));



    if(cx < 0 || cx > width-1 || cy < 0 || cy > height-1) return false;

    //if(cr < 10) return 0;

    //%%% display centre and radius lines.
//    static int divider = 0;
//    if(divider++ % 10000000 == 0) {

//        double mact = 0;
//        cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
//        for(int x = 0; x < 128; x++) {
//            for(int y = 0; y < 128; y++) {
//                double a = activity.queryActivity(x, y);
//                mact = std::max(a, mact);
//                if(a > 0.01)
//                    image2.at<float>(x, y) = a;
//            }
//        }

//        image2 = image2 * (2/mact);
//        cv::Mat i8u(128, 128, CV_8U); image2.copyTo(i8u);
//        cv::Mat image(128, 128, CV_8UC3); cv::cvtColor(i8u, image, CV_GRAY2BGR);
//        cv::resize(image, image, cv::Size(512, 512), 0, 0, CV_INTER_NN);


//        cv::circle(image, cv::Point(p1.y, p1.x)*4, 12, CV_RGB(255, 255, 255));
//        cv::circle(image, cv::Point(p2.y, p2.x)*4, 12, CV_RGB(255, 255, 255));
//        cv::circle(image, cv::Point(p3.y, p3.x)*4, 12, CV_RGB(255, 255, 255));
//        cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(p1.y, p1.x)*4, CV_RGB(255, 255, 255));
//        cv::line(image, cv::Point(p1.y, p1.x)*4, cv::Point(p3.y, p3.x)*4, CV_RGB(255, 255, 255));
//        cv::line(image, cv::Point(p1.y, p1.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
//        cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
//        cv::line(image, cv::Point(p3.y, p3.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
//        cv::circle(image, cv::Point(cy, cx)*4, 5, CV_RGB(255, 0, 0), CV_FILLED);



//        if(divider % 200 == 0) {
//            int xsm = std::max(p1.x-sRadius, 0)*4;
//            int ysm = std::max(p1.y-sRadius, 0)*4;
//            int wsm = std::min(128 - xsm/4, 2*sRadius)*4;
//            int hsm = std::min(128 - ysm/4, 2*sRadius)*4;

//            cv::Mat submat;
//            image(cv::Rect(ysm, xsm, hsm, wsm)).copyTo(submat);
//            cv::flip(submat, submat, 0);
//            cv::resize(submat, submat, cv::Size(0, 0), 4, 4, CV_INTER_NN);
//            cv::flip(image, image, 0);
//            cv::imshow("Local", submat);
//        }


//        cv::imshow("Activity", image);
//        cv::waitKey(1);
//    }

//    cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
//    for(int x = 0; x < 128; x++) {
//        for(int y = 0; y < 128; y++) {
//            image2.at<float>(x, y) = activity.queryActivity(x, y);
//        }
//    }
//    cv::Mat bigim(128*4, 128*4, CV_32F); bigim.setTo(0);
//    cv::resize(image2, bigim, bigim.size());
//    cv::imshow("activity", bigim);
//    cv::waitKey(1);

    //double ca = calculateCircleActivity(cx, cy, cr);
    //if(ca < 1) return 0;


    //std::cout << ca << std::endl;



    return true;

}

