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

void vCircle::trimFilterLocations(int x, int y)
{
    std::vector<act_unit>::iterator i = localActivity.begin();
    while(i != localActivity.end()) {
        if(std::abs(x - i->x) + std::abs(y - i->y) < tRadius)
            localActivity.erase(i);
        else
            i++;
    }
}

bool vCircle::threePointCircle(int x1, int y1, int x2, int y2, int x3, int y3,
                               int &cx, int &cy, double &cr)
{
    if(x2 == x1) {
        if(x2 == x3) {
            std::cerr << "All points found on a straight line" << std::endl;
            return 0;
        }
        else {
            int tempx = x3;
            int tempy = y3;
            x3 = x2; y3 = y2;
            x2 = tempx; y2 = tempy;
        }
    } else if(x3 == x2) {
        int tempx = x1;
        int tempy = y1;
        x1 = x3; y1 = y3;
        x3 = tempx; y3 = tempy;
    }

    //calculate the circle from the 3 points
    double ma = (y2 - y1) / (double)(x2 - x1);
    double mb = (y3 - y2) / (double)(x3 - x2);

    cx = (ma * mb * (y1 - y3) + mb * (x1 - x2) - ma * (x2 + x3)) /
            2 * (mb - ma);
    cy = ma * (cx - x1) + y1;
    cr = sqrt(pow(cx - x1, 2.0) + pow(cy - x1, 2.0));

    if(cx < 0 || cx > width-1 || cy < 0 || cy > height-1) return false;
    return true;

}

emorph::ClusterEvent * vCircle::localCircleEstimate(emorph::AddressEvent &event)
{
    act_unit p1(event.getX(), event.getY(), 0);
    //update the activity here
    p1.a = activity.addEvent(event);

    //create our search locations
    createLocalSearch(p1.x, p1.y);

    //remove points too close to centre
    trimFilterLocations(p1.x, p1.y);
    if(localActivity.size() < 3) return 0;

    //find the best score
    double ba = -1; int bi;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > ba) bi = i;
    act_unit p2 = localActivity[bi];

    //remove activity close-by the first point
    trimFilterLocations(p2.x, p2.y);
    if(localActivity.empty()) return 0;

    //find the second best score
    ba = -1;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > ba) bi = i;
    act_unit p3 = localActivity[bi];

    //make this some sort of curvature test!

    //if we are all on the same y line
    if(p1.y == p2.y && p1.y == p3.y) return 0;

    //ensure p2&p1 and p3&p2 don't lie on the x line
    if(p2.x == p1.x) {
        if(p2.x == p3.x) {
            //std::cerr << "All points found on a straight line" << std::endl;
            return 0;
        }
        else {
            act_unit temp = p3;
            p3 = p2;
            p2 = temp;
        }
    } else if(p3.x == p2.x) {
        act_unit temp = p1;
        p1 = p3;
        p3 = temp;
    }

    // /////////////////////////////////////////////////////////////////////////
    //%%% display activity in the activity window
    cv::Mat image2(128, 128, CV_32F); image2.setTo(0);
    for(int x = 0; x < 128; x++) {
        for(int y = 0; y < 128; y++) {
            image2.at<float>(x, y) = activity.queryActivity(x, y);
        }
    }
    cv::Mat image(512, 512, CV_32F);
    cv::resize(image2, image, image.size());

    cv::circle(image, cv::Point(p1.y, p1.x)*4, 12, CV_RGB(255, 255, 255));
    cv::circle(image, cv::Point(p2.y, p2.x)*4, 12, CV_RGB(255, 255, 255));
    cv::circle(image, cv::Point(p3.y, p3.x)*4, 12, CV_RGB(255, 255, 255));
    cv::imshow("Local Activity", image);
    cv::waitKey(1);
    // /////////////////////////////////////////////////////////////////////////

    //calculate the circle from the 3 points
    double ma = (p2.y - p1.y) / (double)(p2.x - p1.x);
    double mb = (p3.y - p2.y) / (double)(p3.x - p2.x);

    unsigned char cx = (ma * mb * (p1.y - p3.y) + mb * (p1.x - p2.x) - ma * (p2.x + p3.x)) /
            2 * (mb - ma);
    unsigned char cy = ma * (cx - p1.x) + p1.y;
    double cr = sqrt(pow(cx - p1.x, 2.0) + pow(cy - p1.y, 2.0));



    if(cx < 0 || cx > width-1 || cy < 0 || cy > height-1) return 0;


    //%%% display centre and radius lines.
    cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(p1.y, p1.x)*4, CV_RGB(255, 255, 255));
    cv::line(image, cv::Point(p2.y, p2.x)*4, cv::Point(p3.y, p3.x)*4, CV_RGB(255, 255, 255));
    cv::line(image, cv::Point(p1.y, p1.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));
    cv::line(image, cv::Point(p3.y, p3.x)*4, cv::Point(cy, cx)*4, CV_RGB(255, 255, 255));

    cv::imshow("Local Activity", image);
    cv::waitKey(100);



    emorph::ClusterEvent *v = new emorph::ClusterEvent;
    v->setChannel(event.getChannel());
    v->setPolarity(event.getPolarity());
    v->setStamp(event.getStamp());
    v->setXCog(cx);
    v->setYCog(cy);
    //std::cout << (int)cx << " " << (int)cy << std::endl;
    return v;

}

