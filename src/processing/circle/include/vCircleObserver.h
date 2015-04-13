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
#include <iCub/ctrl/kalman.h>

class vCircleObserver
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

    vCircleObserver(int width = 128, int height = 128,
            int sRadius = 16, int tRadius = 8,
            double aDecay = 1000, double aInject = 5, int aRegion = 0)
        : sRadius(sRadius), width(width), height(height), tRadius(tRadius) {

        stepbystep = false;
        activity = emorph::activityMat(height, width, aDecay, aInject, aRegion);
    }

    bool localCircleEstimate(emorph::AddressEvent &event, double &cx,
                             double &cy, double &cr, bool showDebug = false);


    bool threePointCircle(int x1, int y1, int x2, int y2, int x3, int y3,
                          int &cx, int &cy, double &r);

    //temporary debug stuff
    bool stepbystep;
};

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

class vCircleTracker
{
public:

    iCub::ctrl::Kalman *filter;
    emorph::vQueue zq;
    bool active;
    emorph::vtsHelper unwrap;
    double pTS;

    //system variance
    double svPos;
    double svSiz;

    //private functions
    bool makeObservation(double &cx, double &cy, double &cr);
    double Pvgd(double xv, double yv);

public:

    vCircleTracker(double svPos, double svSiz, double zvPos, double zvSiz);
    ~vCircleTracker();


    //add event will:
    //1. check closeness to distribution
    //2. update zq if < c
    //3. perform correction based on observation from zq
    //4. destroy if validation gate too high
    //5. return whether, updated, non-updated, needs destroying
    double addEvent(emorph::AddressEvent v, double cT);

    //update state
    //also update zq values based on velocity
    double predict(double dt);


};


