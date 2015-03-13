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
#include <opencv2/opencv.hpp>

void vCircle::createLocalSearch(int x, int y)
{
    localActivity.clear();
    int tminx = std::max(x-sRadius, 0);
    int tmaxx = std::min(x+sRadius, width -1);
    int tminy = std::max(y-sRadius, 0);
    int tmaxy = std::min(y+sRadius, height-1);

    for(int u = tminx; u <= tmaxx; u++) {
        localActivity.push_back(act_unit(u, tminy, activity.queryActivity(u, tminy)));
        localActivity.push_back(act_unit(u, tmaxy, activity.queryActivity(u, tmaxy)));
    }
    for(int v = tminy+1; v <= tmaxy-1; v++) {
        localActivity.push_back(act_unit(tminx, v, activity.queryActivity(tminx, v)));
        localActivity.push_back(act_unit(tmaxx, v, activity.queryActivity(tmaxx, v)));
    }

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

emorph::ClusterEvent * vCircle::localCircleEstimate(emorph::AddressEvent &event)
{
    act_unit p1(event.getX(), event.getY(), 0);
    //update the activity here
    p1.a = activity.addEvent(event);

    // /////////////////////////////////////////////////////////////////////////
    //%%% display activity in the activity window
    cv::Mat image(sRadius*9, sRadius*9, CV_8U); image.setTo(0);

    // /////////////////////////////////////////////////////////////////////////
    //create our search locations
    createLocalSearch(p1.x, p1.y);

    //%%% display search positions

    //find the best score
    double ba = -1; int bi;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > ba) bi = i;
    act_unit p2 = localActivity[bi];

    //remove activity close-by the first point
    trimFilterLocations(p2.x, p2.y);

    //find the second best score
    ba = -1;
    for(int i = 0; i < localActivity.size(); i++)
        if(localActivity[i].a > ba) bi = i;
    act_unit p3 = localActivity[bi];

    //%%% display first and second best score positions

    //ensure p2&p1 and p3&p2 don't lie on the x line
    if(p2.x == p1.x) {
        if(p2.x == p3.x) {
            std::cerr << "All points found on a straight line" << std::endl;
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

    if(p1.a < 1 || p2.a < 1 || p3.a < 1) return 0;

    //calculate the circle from the 3 points
    double ma = (p2.y - p1.y) / (double)(p2.x - p1.x);
    double mb = (p3.y - p2.y) / (double)(p3.x - p2.x);

    unsigned char cx = (ma * mb * (p1.y - p3.y) + mb * (p1.x - p2.x) - ma * (p2.x + p3.x)) /
            2 * (mb - ma);
    unsigned char cy = ma * (cx - p1.x) + p1.y;
    double cr = sqrt(pow(cx - p1.x, 2.0) + pow(cy - p1.y, 2.0));

    //%%% display centre and radius lines.

    if(cx < 0 || cx > width-1 || cy < 0 || cy > height-1) return 0;

    emorph::ClusterEvent *v = new emorph::ClusterEvent;
    v->setChannel(event.getChannel());
    v->setPolarity(event.getPolarity());
    v->setStamp(event.getStamp());
    v->setXCog(cx);
    v->setYCog(cy);
    //std::cout << (int)cx << " " << (int)cy << std::endl;
    return v;

}

