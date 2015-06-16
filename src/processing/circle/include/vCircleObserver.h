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
    emorph::vWindow *window;

    //parameters
    int spatialRadius;
    int inlierThreshold;
    double angleThreshold;
    double radiusThreshold;

public:

    vCircleObserver();

    void init(int width, int height, int tempWin, int spatialRadius,
              int inlierThresh, double angleThresh, double radThresh);

    bool calculateCircle(double x1, double x2, double x3,
                         double y1, double y2, double y3,
                         double &cx, double &cy, double &cr);

    void addEvent(emorph::vEvent &event);
    //double RANSAC(double &cx, double &cy, double &cr);
    int flowcircle(double &cx, double &cy, double &cr);
    int flowView();


};

/*//////////////////////////////////////////////////////////////////////////////
  CIRCLE TRACKER
  ////////////////////////////////////////////////////////////////////////////*/

class vCircleTracker
{
private:

    iCub::ctrl::Kalman *filter;
    bool active;

    //system variance
    double svPos;
    double svSiz;

    //private functions
    //double Pvgd(double xv, double yv);

public:

    vCircleTracker();
    ~vCircleTracker();

    void init(double svPos, double svSiz, double zvPos, double zvSiz);

    bool startTracking(double xz, double yz, double rz);

    double predict(double dt);
    bool correct(double xz, double yz, double rz);
    bool getState(double &x, double &y, double &r);
    double Pzgd(double xz, double yz, double rz);

    bool isActive() { return active; }


};


