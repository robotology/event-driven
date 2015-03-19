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

#include <iCub/emorph/all.h>
#include <opencv2/opencv.hpp>

class vCircle
{

private:

    //data
    emorph::activityMat activity;


    struct act_unit {
        int x;
        int y;
        double a;
        act_unit(int x, int y, double a): x(x), y(y), a(a) {}
    };
    std::vector<act_unit> localActivity;

    //parameters
    int width;
    int height;
    int sRadius;
    int tRadius;

    //private functions
    void createLocalSearch(int x, int y);
    void pointTrim(int x, int y);
    void linearTrim(int x1, int y1, int x2, int y2);
    cv::Mat createLocalActivityWindow(int x, int y);
    double calculateCircleActivity(int cx, int cy, int r);

public:

    vCircle(int width = 128, int height = 128, int sRadius = 16, int tRadius = 16)
        : sRadius(sRadius), width(width), height(height), tRadius(tRadius) {
        activity = emorph::activityMat(height, width, 1000, 5, 0);
    }
    emorph::ClusterEvent *localCircleEstimate(emorph::AddressEvent &event);


    bool threePointCircle(int x1, int y1, int x2, int y2, int x3, int y3,
                          int &cx, int &cy, double &r);
};


